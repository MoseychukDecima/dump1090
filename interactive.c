// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "dump1090.h"
#include <termios.h>
//
// ============================= Utility functions ==========================
//
static uint64_t mstime(void) {
    struct timeval tv;
    uint64_t mst;

    gettimeofday(&tv, NULL);
    mst = ((uint64_t)tv.tv_sec)*1000;
    mst += tv.tv_usec/1000;
    return mst;
}
//
//=========================================================================
//
// Add a new DF structure to the interactive mode linked list
//
void interactiveCreateDF(struct aircraft *a, struct modesMessage *mm) {
    struct stDF *pDF = (struct stDF *) malloc(sizeof(*pDF));

    if (pDF) {
        // Default everything to zero/NULL
        memset(pDF, 0, sizeof(*pDF));

        // Now initialise things
        pDF->seen        = a->seen;
        pDF->llTimestamp = mm->timestampMsg;
        pDF->addr        = mm->addr;
        pDF->pAircraft   = a;
        memcpy(pDF->msg, mm->msg, MODES_LONG_MSG_BYTES);

        if (!pthread_mutex_lock(&Modes.pDF_mutex)) {
            if ((pDF->pNext = Modes.pDF)) {
                Modes.pDF->pPrev = pDF;
            }
            Modes.pDF = pDF;
            pthread_mutex_unlock(&Modes.pDF_mutex);
        } else {
            free(pDF);
        }
    }
}
//
// Remove stale DF's from the interactive mode linked list
//
void interactiveRemoveStaleDF(time_t now) {
    struct stDF *pDF  = NULL;
    struct stDF *prev = NULL;

    // Only fiddle with the DF list if we gain possession of the mutex
    // If we fail to get the mutex we'll get another chance to tidy the
    // DF list in a second or so.
    if (!pthread_mutex_trylock(&Modes.pDF_mutex)) {
        pDF  = Modes.pDF;
        while(pDF) {
            if ((now - pDF->seen) > Modes.interactive_delete_ttl) {
                if (Modes.pDF == pDF) {
                    Modes.pDF = NULL;
                } else {
                    prev->pNext = NULL;
                }

                // All DF's in the list from here onwards will be time
                // expired, so delete them all
                while (pDF) {
                    prev = pDF; pDF = pDF->pNext;
                    free(prev);
                }

            } else {
                prev = pDF; pDF = pDF->pNext;
            }
        }
        pthread_mutex_unlock (&Modes.pDF_mutex);
    }
}

struct stDF *interactiveFindDF(uint32_t addr) {
    struct stDF *pDF = NULL;

    if (!pthread_mutex_lock(&Modes.pDF_mutex)) {
        pDF = Modes.pDF;
        while(pDF) {
            if (pDF->addr == addr) {
                pthread_mutex_unlock (&Modes.pDF_mutex);
                return (pDF);
            }
            pDF = pDF->pNext;
        }
        pthread_mutex_unlock (&Modes.pDF_mutex);
    }
    return (NULL);
}
//
//========================= Interactive mode ===============================
//
// Возвращаем новую структуру самолета для связанного списка интерактивного режима
// самолетов
//
struct aircraft *interactiveCreateAircraft(struct modesMessage *mm) {
    struct aircraft *a = (struct aircraft *) malloc(sizeof(*a));

    // Default everything to zero/NULL
    memset(a, 0, sizeof(*a));

    // Now initialise things that should not be 0/NULL to their defaults
    a->addr = mm->addr;
    a->lat  = a->lon = 0.0;
    memset(a->signalLevel, mm->signalLevel, 8); // First time, initialise everything
                                                // to the first signal strength

    // mm->msgtype 32 is used to represent Mode A/C. These values can never change, so 
    // set them once here during initialisation, and don't bother to set them every 
    // time this ModeA/C is received again in the future
    if (mm->msgtype == 32) {
        int modeC      = ModeAToModeC(mm->modeA | mm->fs);
        a->modeACflags = MODEAC_MSG_FLAG;
        if (modeC < -12) {
            a->modeACflags |= MODEAC_MSG_MODEA_ONLY;
        } else {
            mm->altitude = modeC * 100;
            mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
        }
    }
    return (a);
}
//
//=========================================================================
//
// Return the aircraft with the specified address, or NULL if no aircraft
// exists with this address.
//
struct aircraft *interactiveFindAircraft(uint32_t addr) {
    struct aircraft *a = Modes.aircrafts;

    while(a) {
        if (a->addr == addr) return (a);
        a = a->next;
    }
    return (NULL);
}
//
//=========================================================================
//
// Мы получили ответ в режиме A или C.
//
// Поиск в списке известных самолетов с режимом S и пометка их, если этот режим A/C
// соответствует известным сигналам режима S или высоте (+/- 50 футов).
//
// Воздушное судно, оснащенное режимом S, также может отвечать на запросы SSR в режимах A и C.
// Мы не можем сказать, режим это A или C, поэтому просматриваем весь список самолетов
// ищем совпадения в режиме A (сигнал) и режиме C (высота). Флаг в режиме S
// записывает, что мы получили потенциальный ответ в режиме A или C от этого самолета.
//
// Если самолет реагирует на режим A, то, скорее всего, он реагирует на режим C
// тоже, и наоборот. Таким образом, как только запись в режиме S помечается как
// и флаг режима C, мы можем быть вполне уверены, что этот кадр режима A/C относится к этому
// Самолет режима S.
//
// Режим C чаще конфликтует, чем режим A; Может быть несколько самолетов
// курсируем на эшелоне FL370, но менее вероятно (хотя и не невозможно), что их будет два
// самолет на том же крике. Поэтому отдавайте предпочтение совпадениям записей в режиме А.
//
// Примечание. Теоретически возможно, что самолет будет иметь такое же значение для режима A.
// и режим C. Поэтому мы должны проверить ОБА A И C для КАЖДОГО S.
//
void interactiveUpdateAircraftModeA(struct aircraft *a) {
    struct aircraft *b = Modes.aircrafts;

    while(b) {
        if ((b->modeACflags & MODEAC_MSG_FLAG) == 0) {// skip any fudged ICAO records 

            // If both (a) and (b) have valid squawks...
            if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_SQUAWK_VALID) {
                // ...check for Mode-A == Mode-S Squawk matches
                if (a->modeA == b->modeA) { // If a 'real' Mode-S ICAO exists using this Mode-A Squawk
                    b->modeAcount   = a->messages;
                    b->modeACflags |= MODEAC_MSG_MODEA_HIT;
                    a->modeACflags |= MODEAC_MSG_MODEA_HIT;
                    if ( (b->modeAcount > 0) &&
                       ( (b->modeCcount > 1)
                      || (a->modeACflags & MODEAC_MSG_MODEA_ONLY)) ) // Allow Mode-A only matches if this Mode-A is invalid Mode-C
                        {a->modeACflags |= MODEAC_MSG_MODES_HIT;}    // flag this ModeA/C probably belongs to a known Mode S                    
                }
            }

            // Если и (a), и (b) имеют действительные высоты...
            if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_ALTITUDE_VALID) {
                // ... проверьте совпадения высоты Mode-C == Mode-S
                if (  (a->modeC     == b->modeC    )     // Если «настоящий» режим S ICAO существует на этой высоте режима C
                   || (a->modeC     == b->modeC + 1)     //          or this Mode-C - 100 ft
                   || (a->modeC + 1 == b->modeC    ) ) { //          or this Mode-C + 100 ft
                    b->modeCcount   = a->messages;
                    b->modeACflags |= MODEAC_MSG_MODEC_HIT;
                    a->modeACflags |= MODEAC_MSG_MODEC_HIT;
                    if ( (b->modeAcount > 0) &&
                         (b->modeCcount > 1) )
                        {a->modeACflags |= (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD);} // отметьте, что этот режим ModeA/C, вероятно, принадлежит известному режиму S                   
                }
            }
        }
        b = b->next;
    }
}
//
//=========================================================================
//
void interactiveUpdateAircraftModeS() {
    struct aircraft *a = Modes.aircrafts;

    while(a) {
        int flags = a->modeACflags;
        if (flags & MODEAC_MSG_FLAG) { // find any fudged ICAO records

            // clear the current A,C and S hit bits ready for this attempt
            a->modeACflags = flags & ~(MODEAC_MSG_MODEA_HIT | MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODES_HIT);

            interactiveUpdateAircraftModeA(a);  // and attempt to match them with Mode-S
        }
        a = a->next;
    }
}
//
//=========================================================================
//
// Receive new messages and populate the interactive mode with more info
//
struct aircraft *interactiveReceiveData(struct modesMessage *mm) {
    struct aircraft *a, *aux;

    // Return if (checking crc) AND (not crcok) AND (not fixed)
    if (Modes.check_crc && (mm->crcok == 0) && (mm->correctedbits == 0))
        return NULL;

    // Lookup our aircraft or create a new one
    a = interactiveFindAircraft(mm->addr);
    if (!a) {                              // If it's a currently unknown aircraft....
        a = interactiveCreateAircraft(mm); // ., create a new record for it,
        a->next = Modes.aircrafts;         // .. and put it at the head of the list
        Modes.aircrafts = a;
    } else {
        /* If it is an already known aircraft, move it on head
         * so we keep aircrafts ordered by received message time.
         *
         * However move it on head only if at least one second elapsed
         * since the aircraft that is currently on head sent a message,
         * othewise with multiple aircrafts at the same time we have an
         * useless shuffle of positions on the screen. */
        if (0 && Modes.aircrafts != a && (time(NULL) - a->seen) >= 1) {
            aux = Modes.aircrafts;
            while(aux->next != a) aux = aux->next;
            /* Now we are a node before the aircraft to remove. */
            aux->next = aux->next->next; /* removed. */
            /* Add on head */
            a->next = Modes.aircrafts;
            Modes.aircrafts = a;
        }
    }

    a->signalLevel[a->messages & 7] = mm->signalLevel;// replace the 8th oldest signal strength
    a->seen      = time(NULL);
    a->timestamp = mm->timestampMsg;
    a->messages++;

    // If a (new) CALLSIGN has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_CALLSIGN_VALID) {
        memcpy(a->flight, mm->flight, sizeof(a->flight));
    }

    // Если получена (новая) ВЫСОТА, скопируйте ее в структуру самолета.
    if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
        if ( (a->modeCcount)                   // if we've a modeCcount already
          && (a->altitude  != mm->altitude ) ) // and Altitude has changed
//        && (a->modeC     != mm->modeC + 1)   // and Altitude not changed by +100 feet
//        && (a->modeC + 1 != mm->modeC    ) ) // and Altitude not changes by -100 feet
            {
            a->modeCcount   = 0;               //....zero the hit count
            a->modeACflags &= ~MODEAC_MSG_MODEC_HIT;
            }
        a->altitude = mm->altitude;
        a->modeC    = (mm->altitude + 49) / 100;
    }

    // If a (new) SQUAWK has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
        if (a->modeA != mm->modeA) {
            a->modeAcount   = 0; // Squawk has changed, so zero the hit count
            a->modeACflags &= ~MODEAC_MSG_MODEA_HIT;
        }
        a->modeA = mm->modeA;
    }

    // Если получен (новый) HEADING, скопируйте его в структуру самолета.
    if (mm->bFlags & MODES_ACFLAGS_HEADING_VALID) {
        a->track = mm->heading;
    }

    // If a (new) SPEED has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_SPEED_VALID) {
        a->speed = mm->velocity;
    }

    // If a (new) Vertical Descent rate has been received, copy it to the aircraft structure
    if (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) {
        a->vert_rate = mm->vert_rate;
    }

    // if the Aircraft has landed or taken off since the last message, clear the even/odd CPR flags
    if ((mm->bFlags & MODES_ACFLAGS_AOG_VALID) && ((a->bFlags ^ mm->bFlags) & MODES_ACFLAGS_AOG)) {
        a->bFlags &= ~(MODES_ACFLAGS_LLBOTH_VALID | MODES_ACFLAGS_AOG);
    }

    // If we've got a new cprlat or cprlon
    if (mm->bFlags & MODES_ACFLAGS_LLEITHER_VALID) {
        int location_ok = 0;

        if (mm->bFlags & MODES_ACFLAGS_LLODD_VALID) {
            a->odd_cprlat  = mm->raw_latitude;
            a->odd_cprlon  = mm->raw_longitude;
            a->odd_cprtime = mstime();
        } else {
            a->even_cprlat  = mm->raw_latitude;
            a->even_cprlon  = mm->raw_longitude;
            a->even_cprtime = mstime();
        }

        // If we have enough recent data, try global CPR
        if (((mm->bFlags | a->bFlags) & MODES_ACFLAGS_LLEITHER_VALID) == MODES_ACFLAGS_LLBOTH_VALID && abs((int)(a->even_cprtime - a->odd_cprtime)) <= 10000) {
            if (decodeCPR(a, (mm->bFlags & MODES_ACFLAGS_LLODD_VALID), (mm->bFlags & MODES_ACFLAGS_AOG)) == 0) {
                location_ok = 1;
            }
        }

        // Otherwise try relative CPR.
        if (!location_ok && decodeCPRrelative(a, (mm->bFlags & MODES_ACFLAGS_LLODD_VALID), (mm->bFlags & MODES_ACFLAGS_AOG)) == 0) {
            location_ok = 1;
        }

        //If we sucessfully decoded, back copy the results to mm so that we can print them in list output
        if (location_ok) {
            mm->bFlags |= MODES_ACFLAGS_LATLON_VALID;
            mm->fLat    = a->lat;
            mm->fLon    = a->lon;
        }
    }

    // Update the aircrafts a->bFlags to reflect the newly received mm->bFlags;
    a->bFlags |= mm->bFlags;

    if (mm->msgtype == 32) {
        int flags = a->modeACflags;
        if ((flags & (MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODEC_OLD)) == MODEAC_MSG_MODEC_OLD) {
            //
            // This Mode-C doesn't currently hit any known Mode-S, but it used to because MODEAC_MSG_MODEC_OLD is
            // set  So the aircraft it used to match has either changed altitude, or gone out of our receiver range
            //
            // We've now received this Mode-A/C again, so it must be a new aircraft. It could be another aircraft
            // at the same Mode-C altitude, or it could be a new airctraft with a new Mods-A squawk.
            //
            // To avoid masking this aircraft from the interactive display, clear the MODEAC_MSG_MODES_OLD flag
            // and set messages to 1;
            //
            a->modeACflags = flags & ~MODEAC_MSG_MODEC_OLD;
            a->messages    = 1;
        }
    }

    // If we are Logging DF's, and it's not a Mode A/C
    if ((Modes.bEnableDFLogging) && (mm->msgtype < 32)) {
        interactiveCreateDF(a,mm);
    }

    return (a);
}
//
//=========================================================================
//
// Show the currently captured interactive data on screen.
//
void interactiveShowData(void) 
{
    struct aircraft *a = Modes.aircrafts;
    time_t now = time(NULL);
    int count = 0;
    char progress;
    char spinner[4] = "|/-\\";

    // Refresh screen every (MODES_INTERACTIVE_REFRESH_TIME) miliseconde
    if ((mstime() - Modes.interactive_last_update) < MODES_INTERACTIVE_REFRESH_TIME)
       {return;}

    Modes.interactive_last_update = mstime();

    // Attempt to reconsile any ModeA/C with known Mode-S
    // We can't condition on Modes.modeac because ModeA/C could be comming
    // in from a raw input port which we can't turn off.
    interactiveUpdateAircraftModeS();

    progress = spinner[time(NULL)%4];

#ifndef _WIN32
    printf("\x1b[H\x1b[2J");    // Clear the screen
#else
    cls();
#endif

    if (Modes.interactive_rtl1090 == 0) {
        printf (
"Hex     Mode  Sqwk  Flight   Alt    Spd  Hdg    Lat      Long   Sig  Msgs   Ti%c\n", progress);
    } else {
        printf (
"Hex    Flight   Alt      V/S GS  TT  SSR  G*456^ Msgs    Seen %c\n", progress);
    }
    printf(
"-------------------------------------------------------------------------------\n");
	struct ToDUMP1090 sendBuf;
	int send_time=0;
    while(a && (count < Modes.interactive_rows)) 
	{

        if ((now - a->seen) < Modes.interactive_display_ttl)
            {
            int msgs  = a->messages;
            int flags = a->modeACflags;

            if ( (((flags & (MODEAC_MSG_FLAG                             )) == 0                    )                 )
              || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEA_ONLY)) == MODEAC_MSG_MODEA_ONLY) && (msgs > 4  ) ) 
              || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD )) == 0                    ) && (msgs > 127) ) 
              ) {
                int altitude = a->altitude, speed = a->speed;
                char strSquawk[5] = " ";
                char strFl[6]     = " ";
                char strTt[5]     = " ";
                char strGs[5]     = " ";

                // Преобразуем единицы измерения в метрику, если указан параметр --metric
                //if (Modes.metric) {
                    altitude = (int) (altitude / 3.2828);  // Футы = метры × 3,28084.
                    speed    = (int) (speed    * 1.852);   // километры в час
                //}

                if (a->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
                    snprintf(strSquawk,5,"%04x", a->modeA);}

                if (a->bFlags & MODES_ACFLAGS_SPEED_VALID) {
                    snprintf (strGs, 5,"%3d", speed);}

                if (a->bFlags & MODES_ACFLAGS_HEADING_VALID) {
                    snprintf (strTt, 5,"%03d", a->track);}

                if (msgs > 99999) {
                    msgs = 99999;}

                if (Modes.interactive_rtl1090) { // RTL1090 display mode

                    if (a->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
                        snprintf(strFl,6,"F%03d",(altitude/100));
                    }
                    printf("%06x %-8s %-4s         %-3s %-3s %4s        %-6d  %-2d\n", 
                    a->addr, a->flight, strFl, strGs, strTt, strSquawk, msgs, (int)(now - a->seen));

                }
				else 
				{                         // Dump1090 display mode
                    char strMode[5]               = "    ";
                    char strLat[8]                = " ";
                    char strLon[9]                = " ";
                    unsigned char * pSig       = a->signalLevel;
                    unsigned int signalAverage = (pSig[0] + pSig[1] + pSig[2] + pSig[3] + 
                                                  pSig[4] + pSig[5] + pSig[6] + pSig[7] + 3) >> 3; 

                    if ((flags & MODEAC_MSG_FLAG) == 0) {
                        strMode[0] = 'S';
                    } else if (flags & MODEAC_MSG_MODEA_ONLY) {
                        strMode[0] = 'A';
                    }
                    if (flags & MODEAC_MSG_MODEA_HIT) {strMode[2] = 'a';}
                    if (flags & MODEAC_MSG_MODEC_HIT) {strMode[3] = 'c';}

                    if (a->bFlags & MODES_ACFLAGS_LATLON_VALID) {
                        snprintf(strLat, 8,"%7.03f", a->lat);
                        snprintf(strLon, 9,"%8.03f", a->lon);
                    }

                    if (a->bFlags & MODES_ACFLAGS_AOG) {
                        snprintf(strFl, 6," grnd");
                    } else if (a->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
                        snprintf(strFl, 6, "%5d", altitude);
                    }


                    int serial_port = open("/dev/ttyS0", O_RDWR); //  
                    struct termios tty;

                    if (tcgetattr(serial_port, &tty) != 0)
                    {
                       printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
                    }
                    // настройки порта 
                        tty.c_cflag &= ~PARENB;   // Clear parity bit, disabling parity (most common)
                        tty.c_cflag &= ~CSTOPB;   // Clear stop field, only one stop bit used in communication (most common)
                        tty.c_cflag &= ~CSIZE;    // Clear all the size bits, then use one of the statements bel
                        tty.c_cflag |= CS8;       // 8 bits per byte (most common)
                        tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
                        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

                        tty.c_lflag &= ~ICANON;   //Canonical mode is disabled with:
                        tty.c_lflag &= ~ECHO;     // Disable echo
                        tty.c_lflag &= ~ECHOE;    // Disable erasure
                        tty.c_lflag &= ~ECHONL;   // Disable new-line echo
                        tty.c_lflag &= ~ISIG;     // Disable interpretation of INTR, QUIT and SUSP
                        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
                        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

                        tty.c_oflag &= ~OPOST;   // Prevent special interpretation of output bytes (e.g. newline chars)
                        tty.c_oflag &= ~ONLCR;   // Prevent conversion of newline to carriage return/line feed 

                        tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
                        tty.c_cc[VMIN] = 0;

                        cfsetispeed(&tty, B115200);
                        cfsetospeed(&tty, B115200);

                    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
                    {
                       printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                    }

					struct ToDUMP1090 sendBuf;
					memset(&sendBuf,0, sizeof(sendBuf)); // Очистить массив
					
					sendBuf.addr = a->addr;                                   // ICAO address
					memcpy(sendBuf.squawk,strSquawk, sizeof(strSquawk));      // Flight number
					memcpy(sendBuf.flight,a->flight, sizeof(sendBuf.flight)); // номер рейса
					sendBuf.altitude = altitude;                              // Altitude метры
					sendBuf.speed = speed;                                    // Скорость км/час
					sendBuf.track = a->track;                                 // курс в градусах
					sendBuf.vert_rate = a->vert_rate;                         // скорость подъема/снижения
					sendBuf.lat = (float)a->lat;
					sendBuf.lon = (float)a->lon;
					sendBuf.seen_time = (int)(now - a->seen);                  // Время получения последнего пакета
					memcpy(sendBuf.endOfPacket, "\xFF\xFF\xFF", 3);
								
					if((int)(now - a->seen) < 55)
					{
					   write(serial_port, (void*)&sendBuf, sizeof(sendBuf));
					   memset(&sendBuf,0, sizeof(sendBuf)); // Очистить массив
					}

	                close(serial_port);
                }
                count++;
            }
        }
      a = a->next;
    }
}
//
//=========================================================================
//
// When in interactive mode If we don't receive new nessages within
// MODES_INTERACTIVE_DELETE_TTL seconds we remove the aircraft from the list.
//
void interactiveRemoveStaleAircrafts(void) {
    struct aircraft *a = Modes.aircrafts;
    struct aircraft *prev = NULL;
    time_t now = time(NULL);

    // Only do cleanup once per second
    if (Modes.last_cleanup_time != now) {
        Modes.last_cleanup_time = now;

        interactiveRemoveStaleDF(now);

        while(a) {
            if ((now - a->seen) > Modes.interactive_delete_ttl) {
                // Remove the element from the linked list, with care
                // if we are removing the first element
                if (!prev) {
                    Modes.aircrafts = a->next; free(a); a = Modes.aircrafts;
                } else {
                    prev->next = a->next; free(a); a = prev->next;
                }
            } else {
                prev = a; a = a->next;
            }
        }
    }
}
//
//=========================================================================
//
