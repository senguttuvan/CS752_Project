/*
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Swetha 
 *          Navin
 */

/**
 * @file
 * Markov Prefetcher template instantiations.
 */

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/markov.hh"

void
MarkovPrefetcher::calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
                                    std::list<Cycles> &delays)
{

	if (!pkt->req->hasPC()) {
        DPRINTF(HWPrefetch, "ignoring request with no PC");
        return;
    }

    Addr data_addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
    MasterID master_id = useMasterId ? pkt->req->masterId() : 0;
    Addr pc = pkt->req->getPC();
    assert(master_id < Max_Contexts);
    std::list<MarkovEntry*> &tab = table[master_id]; 
	
     // Print Table
    std::list<MarkovEntry*>::iterator print_iter;
    DPRINTF(HWPrefetch,"Miss add          Pre1        Pre2   \n");
    for (print_iter = tab.begin(); print_iter != tab.end(); print_iter++) {
         DPRINTF(HWPrefetch, "%x \t %x \t %x\n",(*print_iter)->missAddr,(*print_iter)->Pre_miss1, (*print_iter)->Pre_miss2);

    }
    // Revert to simple N-block ahead prefetch for instruction fetches
    if (instTagged && pkt->req->isInstFetch()) {
        for (int d = 1; d <= degree; d++) {
            Addr new_addr = data_addr + d * blkSize;
            if (pageStop && !samePage(data_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            }
            DPRINTF(HWPrefetch, "queuing prefetch to %x @ %d\n",new_addr, latency);
            addresses.push_back(new_addr);
            delays.push_back(latency);
        }
        return;
    }

    /* Scan Table for instAddr Match */
    std::list<MarkovEntry*>::iterator iter;
    for (iter = tab.begin(); iter != tab.end(); iter++) {
        // Entries have to match on the security state as well
        if ((*iter)->missAddr == data_addr && (*iter)->isSecure == is_secure)
            break;
    } 

    if (iter != tab.end()) {
        // Hit in table

        if ((*iter)->confidence < Max_Conf)
        {        (*iter)->confidence++;
        }
 	Addr prefetch1 = (*iter)-> Pre_miss1;
	Addr prefetch2 = (*iter)-> Pre_miss2;
	if (prefetch1 != 0){
 		addresses.push_back(prefetch1);
        	delays.push_back(latency);
	}
	if(prefetch2 != 0){
 		addresses.push_back(prefetch2);
       		delays.push_back(latency);
	}

        DPRINTF(HWPrefetch, "hit: PC %x data_addr %x (%s) Prefetch 1 %x , Prefetch 2 %x, Confidence %d\n", pc, data_addr, is_secure ? "s" : "ns",  prefetch1, prefetch2, (*iter)->confidence);

    } else {
        // Miss in table
        // Find lowest confidence and replace

        DPRINTF(HWPrefetch, "miss: PC %x data_addr %x (%s)\n", pc, data_addr,
                is_secure ? "s" : "ns");

        if (tab.size() >= 256) { 
            std::list<MarkovEntry*>::iterator min_pos = tab.begin();
            int min_conf = (*min_pos)->confidence;
            for (iter = min_pos, ++iter; iter != tab.end(); ++iter) {
                if ((*iter)->confidence < min_conf){
                    min_pos = iter;
                    min_conf = (*iter)->confidence;
                }
            }
            DPRINTF(HWPrefetch, "  replacing PC %x (%s)\n",(*min_pos)->missAddr, (*min_pos)->isSecure ? "s" : "ns");

            delete *min_pos;
            tab.erase(min_pos);
        }

        MarkovEntry *new_entry = new MarkovEntry;
        new_entry->missAddr = data_addr;
        new_entry->isSecure = is_secure;
        new_entry->confidence = 0;
	new_entry->Pre_miss1 = 0;
	new_entry->Pre_miss2 = 0;
        tab.push_back(new_entry);
    }
    //Update the history information of the previous miss address 
    if(Previous_missaddr != 0)
    {
        for (iter = tab.begin(); iter != tab.end(); iter++) {
        // Entries have to match on the security state as well
           if ((*iter)->missAddr == Previous_missaddr && (*iter)->isSecure == is_secure)
                break;
        }
	if((*iter)->Pre_miss1 != data_addr && (*iter)->Pre_miss2 != data_addr ){
		if((*iter)->Pre_miss1 == 0) {
			(*iter)->Pre_miss1 = data_addr;
		}
		else if((*iter)->Pre_miss2 == 0 ){
			(*iter)->Pre_miss2 = data_addr;	
        	}
		else {
			(*iter)->Pre_miss1 = (*iter)->Pre_miss2;
	   		(*iter)->Pre_miss2 = data_addr;
        	}
	} 
    }
    Previous_missaddr = data_addr;	

}


MarkovPrefetcher*
MarkovPrefetcherParams::create()
{
   return new MarkovPrefetcher(this);
}
