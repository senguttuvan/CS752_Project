/*
 * Copyright (c) 2015-2016 UW-Madison
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
 * Authors: Navin Senguttuvan
 *          Swetha Srinivasan
 */

/**
 * @file
 * Global History Buffer based Delta Prefetcher template instantiations.
 */

#include "base/trace.hh"
#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/ghb_delta.hh"
#define GHBSIZE 256
#define INDEX_TABLE_SIZE 64

void
GlobalDeltaPrefetcher::calculatePrefetch(PacketPtr &pkt, std::list<Addr> &addresses,
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
    std::vector<IndexTableEntry*> &indexTab = indexTable[master_id];
    std::vector<TableEntry*> &GHBtab = table[master_id];

    DPRINTF(HWPrefetch, "In Delta prefetcher \nPC: %x Data Add: %p ghb size: %d ,index tab size : %d ,master id %d \n",pc,data_addr, GHBtab.size(),indexTab.size(), master_id );

    

    // Revert to simple N-block ahead prefetch for instruction fetches
    if (instTagged && pkt->req->isInstFetch()) {
        for (int d = 1; d <= degree; d++) {
            Addr new_addr = data_addr + d * blkSize;
            if (pageStop && !samePage(data_addr, new_addr)) {
                // Spanned the page, so now stop
                pfSpanPage += degree - d + 1;
                return;
            }
            DPRINTF(HWPrefetch, "GLOBAL HISTORY BUFFER::Markov::queuing prefetch to %x @ %d\n",new_addr, latency);
            addresses.push_back(new_addr);
            delays.push_back(latency);
        }
        return;
    }
 
    int delta = data_addr - Previous_missaddr;

    /* Scan Table for instAddr Match */
    std::vector<IndexTableEntry*>::iterator iter;
    std::vector<TableEntry*>::iterator tabIter;
    std::vector<TableEntry*>::iterator GHB_Pre_iter;

    for (iter = indexTab.begin(); iter != indexTab.end(); iter++) {
        // Entries have to match on the security state as well
        if ((*iter)->key == delta)
            break;
    }

    if (iter != indexTab.end()) {
        // Hit in table
	DPRINTF(HWPrefetch, "Hit in the index table\n");

	if ((*iter)->confidence < Max_Conf ) {
		(*iter)->confidence++;
	}
	
	TableEntry* TabEntry = (*iter)->historyBufferEntry;

	for (tabIter = GHBtab.begin(); tabIter != GHBtab.end(); tabIter++) {
        // Get the corresponding itertor (this is needed to get the next element in the vector)
          if ( (*tabIter) == TabEntry){
		break;
    	  }
        }

	std::vector<TableEntry*>::iterator  tab_Pre_iter = tabIter+1;
	// Traverse the link list to find all possible prefetch address    

	int local_delta = 0;
	for (int i = 0; i < degree - 1; i++) {
        	if (*tab_Pre_iter != NULL) {

       
                	 //Calculate detla betn current and next value in GHB
  		         local_delta += (*tab_Pre_iter)->missAddr - (*tabIter)->missAddr ;

		         Addr new_address =  (data_addr)+ local_delta;

 
	         	 DPRINTF(HWPrefetch, "Delta: %d, local_delta: %d current miss: %p prev_miss addr :%p prefetch address:%x \n",delta,local_delta, data_addr, (*tabIter)->missAddr, new_address );
			addresses.push_back(new_address);
           		 delays.push_back(latency); ///Inserttttttttttttt page span !!!! (Check double correlation)
		} else {
		  break;
		}
		tabIter = tab_Pre_iter;
		tab_Pre_iter++;
	}

    	  // update GHB with the latest miss addr
          if (GHBtab.size() >= GHBSIZE ) // Default GHB size is set to 256
          {
 	      GHBtab.erase(GHBtab.begin());	
          }
        TableEntry* new_entry = new TableEntry;
        new_entry->missAddr = data_addr;
        new_entry->isSecure = is_secure;
        new_entry->listPointer = (*iter)->historyBufferEntry;

        GHBtab.push_back(new_entry);

        DPRINTF(HWPrefetch, "After addin in hit PC: %x Data Add: %p ghb size: %d \n",pc,data_addr, GHBtab.size() );              
	// Update index table to point to the new GHB entry
	(*iter)->historyBufferEntry = GHBtab.back();

     } else {
        // Miss in table

	DPRINTF(HWPrefetch, "Miss in the index table\n");

	// Insert missed addr value into the GHB

	// Check for the size overflow in GHB
        if (GHBtab.size() >= GHBSIZE ) // Default GHB size is set to 256
        {
	DPRINTF(HWPrefetch, "GHB size full in miss\n");
	
 	      GHBtab.erase(GHBtab.begin());	
        }
	TableEntry* new_entry = new TableEntry;
        new_entry->missAddr = data_addr;
        new_entry->isSecure = is_secure;
        new_entry->listPointer = NULL;
	
        GHBtab.push_back(new_entry);
	DPRINTF(HWPrefetch, "After addin in miss PC: %x Data Add: %p ghb size: %d \n",pc,data_addr, GHBtab.size() );

	// Insert a corresponding index table entry
	if (indexTab.size() == INDEX_TABLE_SIZE){
		//Check if any of the listpointers in the Index table is invalid

	      bool invalidFound = false;
	
	      for (iter = indexTab.begin(); iter != indexTab.end(); iter ++ )
        	{
			if ( GHBtab.back() - (*iter)->historyBufferEntry >= GHBSIZE  )
			 {
			    DPRINTF(HWPrefetch, "Invalid ghb entry : GHBtab.back %x, (*iter)->hist %x\n",GHBtab.back(), (*iter)->historyBufferEntry);

			    //Value is invalid
			    indexTab.erase(iter);
			    invalidFound = true;
			    break;
			}
		}
	
        	if(invalidFound)
        	{
 			//Insert into the index table
        		IndexTableEntry *new_entry = new IndexTableEntry;
        		new_entry->key = delta;
			new_entry->confidence = 0;
        		new_entry->historyBufferEntry = GHBtab.back() ;
        	        indexTab.push_back(new_entry);
			DPRINTF(HWPrefetch, "Insert in index when full: %x\n",new_entry->key);

    		}
		else // if it is the end and no invalid values wer found should decide which to replace
		{
        	// Check for confidence if no entry is invalid
		
			int lowest_confidence = Max_Conf;
			std::vector<IndexTableEntry*>::iterator lru_iter;
	
        		for (iter = indexTab.begin(); iter != indexTab.end(); iter ++ )
        		{
				if ( (*iter)->confidence < lowest_confidence )
				{
					// If confidence less than lowest confidence, save the index table entry
					lru_iter = iter;
					lowest_confidence = (*iter)->confidence;
				}
			}
			DPRINTF(HWPrefetch, "Insert in index after evivting a block: miss addr: %x\n",(*lru_iter)->key);

 			indexTab.erase(lru_iter);

			IndexTableEntry *new_entry = new IndexTableEntry;
        		new_entry->key = delta;
			new_entry->confidence = 0;
        		new_entry->historyBufferEntry = GHBtab.back() ;
        	        indexTab.push_back(new_entry);
		}
	} else {

		IndexTableEntry *new_entry = new IndexTableEntry;
        	new_entry->key = delta;
		new_entry->confidence = 0;
        	new_entry->historyBufferEntry = GHBtab.back() ;
                indexTab.push_back(new_entry);
		DPRINTF(HWPrefetch, "Insert in index when index table not full %x\n",new_entry->key);

		}

      } 
}


GlobalDeltaPrefetcher*
GlobalDeltaPrefetcherParams::create()
{
   return new GlobalDeltaPrefetcher(this);
}
