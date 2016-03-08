/*
 * ndn-pit-entry-nrimpl.h
 *
 *  Created on: Jan 21, 2015
 *      Author: chenyishun
 */


#ifndef NDN_PIT_ENTRY_NRIMPL_H_
#define NDN_PIT_ENTRY_NRIMPL_H_

#include "ns3/ndn-pit-entry.h"
#include "ns3/ndn-pit-entry-incoming-face.h"
#include "ns3/ndn-name.h"

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ns3 {
namespace ndn {

class Pit;

namespace pit {
namespace nrndn{
	
	/**
	 * @ingroup ndn-pit
	 * @brief PIT entry implementation with additional pointers to the underlying container
	 *           It is completely unrelated to the class PIT::Entry
	 */

class EntryNrImpl : public Entry
{
public:
	typedef Entry super;

	EntryNrImpl(Pit &container, Ptr<const Interest> header,Ptr<fib::Entry> fibEntry);
	virtual ~EntryNrImpl();
	
	/**
	 * @brief Remove all the timeout event in the timeout event list
	 *
	 */
	//void RemoveAllTimeoutEvent();

	/**
	 * @brief Add `id` to the list of incoming neighbor list(m_incomingnbs)
	 *
	 * @param id Neighbor id to add to the list of incoming neigbhor list
	 * @returns iterator to the added neighbor id
	 */
	std::unordered_set< std::string >::iterator
	AddIncomingNeighbors(std::string lane);

	const std::unordered_set<std::string >& getIncomingnbs() const
	{
		return m_incomingnbs;
	}

	//add by DJ on Jan 4,2016:when receive corresponding data packet,remove the pit entry
	void RemoveIncomingNeighbors(std::string name);
//private:
	//void AddNeighborTimeoutEvent(uint32_t id);

	//when the time expire, the incoming neighbor id will be removed automatically
	//void CleanExpiredIncomingNeighbors(uint32_t id);
	std::string getEntryName()
	{
		return m_interest_name;
	}

	void Print(std::ostream &os) const;
	void setInterestName(std::string name);
private:
	//std::unordered_map< uint32_t,EventId> m_nbTimeoutEvent;///< @brief it is a hashmap that record the timeout event of each neighbor id

	//explanation by DJ on Jan 4,2016:m_incomingnbs means last hop
	//std::unordered_set< uint32_t > 		  m_incomingnbs;///< @brief container for incoming neighbors

	//add by DJ on Jan 4,2016:m_incomingnbs means last hop and its interest packet
	std::unordered_set< std::string > m_incomingnbs;

	std::string m_interest_name;	
	//Time m_infaceTimeout;

};



} // namespace nrndn
} // namespace pit
} // namespace ndn
} // namespace ns3


#endif /* NDN_PIT_ENTRY_NRIMPL_H_ */
