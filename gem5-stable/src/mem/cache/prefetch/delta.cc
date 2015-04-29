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
 * Delta Prefetcher template instantiations.
 */

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/delta.hh"

void
DeltaPrefetcher::calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
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
    std::list<DeltaEntry*> &tab = table[master_id]; 
    int currDist = (Previous_missaddr == 0) ? 0 : Previous_missaddr - data_addr;

     // Print Table
    std::list<DeltaEntry*>::iterator print_iter;
    std::vector<int>::iterator print_vec;


    DPRINTF(HWPrefetch,"\n***CALCULATE PREFETCH*** \n" );
    DPRINTF(HWPrefetch,"Previous Miss Address %x Distance: %d ",  Previous_missaddr, currDist);

    DPRINTF(HWPrefetch,"\n\n\n:::::::::::::: Table for current distance %d ::::::::::::::::::::::: ", currDist);
    for (print_iter = tab.begin(); print_iter != tab.end(); print_iter++) {
	DPRINTF(HWPrefetch,"\n Delta table - distance %d \t", (*print_iter)->distance);
	for (print_vec=(*print_iter)->Pre_distance.begin();print_vec != (*print_iter)->Pre_distance.end(); print_vec++)
         { std::cout<<(*print_vec)<<"\t"; 
         }
	}
    DPRINTF(HWPrefetch,"\n\n\nEnd of Table ::::::::::::::::::::::: \n");



    // Revert to simple N-block ahead prefetch for instruction fetches
    if (instTagged && pkt->req->isInstFetch()) {
        for (int d = 1; d <= degree; d++) {
            Addr new_addr = data_addr + d * blkSize;
            if (pageStop && !samePage(data_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            }
            DPRINTF(HWPrefetch, "Inst tag queuing prefetch to %x @ %d\n",new_addr, latency);
            addresses.push_back(new_addr);
            delays.push_back(latency);
        }
        return;
    }

    /* Scan Table for instAddr Match */
    std::list<DeltaEntry*>::iterator iter;
    std::vector<int>::iterator vec_iter;
    std::list<DeltaEntry*>::iterator pf_width;

    for (iter = tab.begin(); iter != tab.end(); iter++) {
        // Entries have to match on the security state as well
        if ((*iter)->distance == currDist && (*iter)->isSecure == is_secure)
            break;
    } 

    if (iter != tab.end()) {
        // Hit in table
	DPRINTF(HWPrefetch,"Hit in the table for distance %d\n",(*iter)->distance);
        if ((*iter)->confidence < Max_Conf)
        {        (*iter)->confidence++;
        }
  	int deg=0;	
        for(vec_iter = (*iter)->Pre_distance.begin();vec_iter !=  (*iter)->Pre_distance.end();vec_iter++)
	{
		int new_dist = *vec_iter;
		Addr new_addr = data_addr + new_dist ;
		if ( (signed int)new_addr < 0 ){continue;}
		//if (pageStop && !samePage(data_addr, new_addr)) {
                	// Spanned the page, so now stop
              	//        pfSpanPage += degree - deg + 1;
               	//	return;
          	//} else {
			DPRINTF(HWPrefetch,"Prefetched addr in depth D=%d distance=%d is %x\n",deg, new_dist, new_addr);
			addresses.push_back(new_addr);
			delays.push_back(latency);
		//}
		deg =deg+1;
		
		//Prefetch width
/*		for (int degr =0 ; degr< degree-1; degr++){
			for(pf_width = tab.begin();pf_width != tab.end();pf_width++ ){
				if ((*pf_width)->distance == new_dist && (*pf_width)->isSecure == is_secure)
          				  break;
			}
			if(pf_width != tab.end()){
				if(!(*pf_width)->Pre_distance.empty()) {
					
					new_dist = (*pf_width)->Pre_distance.back();
					new_addr += new_dist;
					if ( (signed int)new_addr < 0 ){break;}
					if (pageStop && !samePage(data_addr, new_addr)) {
                				// Spanned the page, so now stop
						
              	    		    		pfSpanPage += degree - degr + 1;
               					return;
          				} else {  // COMMENTED FOR DEBUG
						DPRINTF(HWPrefetch,"Prefetched addr in width D=%d and dist:%d is %x\n",degr,new_dist,new_addr);
						addresses.push_back(new_addr);
						delays.push_back(latency);
					//}  // COMMENTED FOR DEBUG

				}
				else {
					break;
				}
			}
			else {
				break;
			}

		} */
			
	}

        	
        DPRINTF(HWPrefetch, "hit: PC %x data_addr %x (%s) distance %d, Confidence %d\n", pc, data_addr, is_secure ? "s" : "ns",currDist,  (*iter)->confidence);

    } else {
        // Miss in table
        // Find lowest confidence and replace

        DPRINTF(HWPrefetch, "miss: PC %x data_addr %x (%s) dist:%d \n", pc, data_addr, is_secure ? "s" : "ns", currDist);

        if (tab.size() >= 256) { 
            std::list<DeltaEntry*>::iterator min_pos = tab.begin();
            int min_conf = (*min_pos)->confidence;
            for (iter = min_pos, ++iter; iter != tab.end(); ++iter) {
                if ((*iter)->confidence < min_conf){
                    min_pos = iter;
                    min_conf = (*iter)->confidence;
                }
            }
            DPRINTF(HWPrefetch, "  replacing PC %x (%s)\n",(*min_pos)->distance, (*min_pos)->isSecure ? "s" : "ns");

            delete *min_pos;
            tab.erase(min_pos);
        }

	if (currDist != 0){
        	DeltaEntry *new_entry = new DeltaEntry;
	        new_entry->distance = currDist;
        	new_entry->isSecure = is_secure;
	        new_entry->confidence = 0;
        	tab.push_back(new_entry);
	}
    }

    //Update the history information of the previous miss address 
    if(Previous_distance != 0)
    {
        for (iter = tab.begin(); iter != tab.end(); iter++) {
        // Entries have to match on the security state as well
           if ((*iter)->distance == Previous_distance && (*iter)->isSecure == is_secure)
                break;
        }
	if(iter != tab.end()) {
		
		for(vec_iter = (*iter)->Pre_distance.begin();vec_iter !=  (*iter)->Pre_distance.end();vec_iter++ )
		{
			if (currDist == *vec_iter) {
				DPRINTF(HWPrefetch,"Value already exists in table %d",currDist);
				break;
			}
		}
		if(vec_iter != (*iter)->Pre_distance.end())
		{	DPRINTF(HWPrefetch,"Erasing that duplicate value %d",currDist);
			(*iter)->Pre_distance.erase(vec_iter);
		}
		if ((*iter)->Pre_distance.size() >= degree) {
			DPRINTF(HWPrefetch,"Size more than degree so erasing the first value %d",currDist);
			(*iter)->Pre_distance.erase((*iter)->Pre_distance.begin());
		} 
		(*iter)->Pre_distance.push_back(currDist);	
	}
    }
    Previous_distance = currDist;	
    Previous_missaddr = data_addr; 
}


DeltaPrefetcher*
DeltaPrefetcherParams::create()
{
   return new DeltaPrefetcher(this);
}
