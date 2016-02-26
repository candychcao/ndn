/*
 * nrHeader.cc
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#include "tableHeader.h"
#include <set>
#include <string>
#include <unordered_set>
#include <map>

namespace ns3
{
namespace ndn
{
namespace nrndn
{

tableHeader::tableHeader():
		m_sourceId(0)
{
	// TODO Auto-generated constructor stub

}

tableHeader::~tableHeader()
{
	// TODO Auto-generated destructor stub
}

TypeId tableHeader::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::tableHeader")
	    .SetParent<Header> ()
	    .AddConstructor<tableHeader> ()
	    ;
	return tid;
}

TypeId tableHeader::GetInstanceTypeId() const
{
	return GetTypeId ();
}

uint32_t tableHeader::GetSerializedSize() const
{
	uint32_t size=0;
	size += sizeof(m_sourceId);

	//m_pitContainer.size():
	size += sizeof(uint32_t);
	for(uint32_t i = 0; i<m_pitContainer.size(); ++i)
	{
		//m_interest_name.size():
		size += sizeof(uint32_t);
		size += m_pitContainer[i]->getEntryName().size();

		//m_incomingnbs.size():
		size += sizeof(uint32_t);
		std::unordered_set<std::string>::const_iterator it;
		for(it = (m_pitContainer[i]->getIncomingnbs()).begin() ; it != m_pitContainer[i]->getIncomingnbs().end(); ++it)
		{
			size += sizeof(uint32_t);
			size +=(*it).size();
		}
	}

	//m_fibContainer.size():
	size += sizeof(uint32_t);
	for(uint32_t i = 0; i<m_fibContainer.size(); ++i)
	{
		//m_interest_name.size():
		size += sizeof(uint32_t);
		size += m_fibContainer[i]->getEntryName().size();

		//m_incomingnbs.size():
		size += sizeof(uint32_t);
		std::unordered_map<std::string, uint32_t >::const_iterator it;
		for(it = (m_fibContainer[i]->getIncomingnbs()).begin(); it != m_fibContainer[i]->getIncomingnbs().end(); ++it)
		{
			size += sizeof(uint32_t);
			size +=it->first.size();
			size += sizeof(uint32_t);
		}
	}
	return size;
}

void tableHeader::Serialize(Buffer::Iterator start) const
{
	Buffer::Iterator& i = start;
	i.WriteHtonU32(m_sourceId);

	i.WriteHtonU32(m_pitContainer.size());
	for(uint32_t it = 0; it<m_pitContainer.size(); ++it)
	{
		i.WriteHtonU32(m_pitContainer[it]->getEntryName().size());
		for(uint32_t j = 0; j<m_pitContainer[it]->getEntryName().size(); ++j)
				i.Write((uint8_t*)&((m_pitContainer[it]->getEntryName())[j]),sizeof(char));

		i.WriteHtonU32(m_pitContainer[it]->getIncomingnbs().size());
		std::unordered_set<std::string>::const_iterator j;
		for(j = (m_pitContainer[it]->getIncomingnbs()).begin(); j != m_pitContainer[it]->getIncomingnbs().end(); ++j)
		{
			i.WriteHtonU32((*j).size());
			for(uint32_t k = 0; k<(*j).size(); ++k)
					i.Write((uint8_t*)&((*j)[k]),sizeof(char));
		}
	}

	i.WriteHtonU32(m_fibContainer.size());
	for(uint32_t it = 0; it<m_fibContainer.size(); ++it)
	{
		i.WriteHtonU32(m_fibContainer[it]->getEntryName().size());
		for(uint32_t j = 0; j<m_fibContainer[it]->getEntryName().size(); ++j)
				i.Write((uint8_t*)&((m_fibContainer[it]->getEntryName())[j]),sizeof(char));

		i.WriteHtonU32(m_fibContainer[it]->getIncomingnbs().size());
		std::unordered_map<std::string, uint32_t >::const_iterator j;
		for(j = (m_fibContainer[it]->getIncomingnbs()).begin(); j != m_fibContainer[it]->getIncomingnbs().end(); ++j)
		{
			i.WriteHtonU32(j->first.size());
			for(uint32_t k = 0; k<j->first.size(); ++k)
						i.Write((uint8_t*)&((j->first)[k]),sizeof(char));
			i.WriteHtonU32(j->second);
		}
	}
}

uint32_t tableHeader::Deserialize(Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	m_sourceId	=	i.ReadNtohU32();

	m_pitContainer.clear();
	uint32_t size  = i.ReadNtohU32();
	for (uint32_t j = 0; j < size; j++)
	{
		char tempchar;
		std::string tempstring;

		Ptr<pit::nrndn::EntryNrImpl> temp;
		uint32_t namesize =  i.ReadNtohU32();
		for(uint32_t k = 0; k < namesize; ++k)
		{
			i.Read((uint8_t*)&(tempchar),sizeof(char));
			tempstring += tempchar;
		}
		//temp->setinterestname(tempstring);

		std::unordered_set< std::string > tempnb;
		uint32_t nbsize =  i.ReadNtohU32();
		for(uint32_t k = 0; k < nbsize; ++k)
		{
			tempstring = "";
			namesize =  i.ReadNtohU32();
			for(uint32_t p = 0; p < namesize; ++p)
			{
				i.Read((uint8_t*)&(tempchar),sizeof(char));
				tempstring += tempchar;
			}
			tempnb.insert(tempstring);
		}
		//temp->setnb(tempnb);
		m_pitContainer.push_back(temp);
	}

	m_fibContainer.clear();
	size = i.ReadNtohU32();
	for (uint32_t j = 0; j < size; j++)
	{
		char tempchar;
		std::string tempstring;

		Ptr<fib::nrndn::EntryNrImpl> temp;
		uint32_t namesize =  i.ReadNtohU32();
		for(uint32_t k = 0; k < namesize; ++k)
		{
			i.Read((uint8_t*)&(tempchar),sizeof(char));
			tempstring += tempchar;
		}
		//temp->setinterestname(tempstring);

		std::unordered_map< std::string,uint32_t > tempnb;
		uint32_t nbsize =  i.ReadNtohU32();
		for(uint32_t k = 0; k < nbsize; ++k)
		{
			uint32_t tempttl;
			tempstring = "";
			namesize =  i.ReadNtohU32();
			for(uint32_t p = 0; p < namesize; ++p)
			{
				i.Read((uint8_t*)&(tempchar),sizeof(char));
				tempstring += tempchar;
			}
			tempttl =  i.ReadNtohU32();
			tempnb.insert(make_pair(tempstring,tempttl));
		}
		//temp->setnb(tempnb);
		m_fibContainer.push_back(temp);
	}

	uint32_t dist = i.GetDistanceFrom(start);
	NS_ASSERT(dist == GetSerializedSize());
	return dist;
}

void tableHeader::Print(std::ostream& os) const
{
	//os<<"nrHeader conatin: NodeID="<<m_sourceId<<"\t coordinate=("<<m_x<<","<<m_y<<") priorityList=";
	std::vector<uint32_t>::const_iterator it;

	os<<std::endl;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
