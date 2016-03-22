/*
 * nrHeader.cc
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/core-module.h"
#include "ns3/ndn-pit.h"
#include "ns3/ndn-fib.h"
#include "ns3/log.h"

#include "ndn-pit-entry-nrimpl.h"
#include "ndn-fib-entry-nrimpl.h"
#include "ndn-nr-pit-impl.h"
#include "ndn-nr-fib-impl.h"
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

NS_OBJECT_ENSURE_REGISTERED (tableHeader);

tableHeader::tableHeader():
		m_sourceId(0),
		m_signature(0)
{
	// TODO Auto-generated constructor stub
	if(m_fib == 0)
	{
		m_fib= CreateObject<fib::nrndn::NrFibImpl>();
		//if(fib)
			//	 	m_fib = DynamicCast<fib::nrndn::NrFibImpl>(fib);
	}

	if(m_pit == 0)
	{
		m_pit =  CreateObject<pit::nrndn::NrPitImpl>();
	}
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

	    .AddAttribute ("Pit","pit of table",
	    		  PointerValue (),
		    	  MakePointerAccessor (& tableHeader::m_pit),
		    	  MakePointerChecker<ns3::ndn::pit::nrndn::NrPitImpl> ())
	    .AddAttribute ("Fib","fib of table",
	    	  PointerValue (),
		    	  MakePointerAccessor (& tableHeader::m_fib),
		    	  MakePointerChecker<ns3::ndn::fib::nrndn::NrFibImpl> ())
	    ;
	return tid;
}

TypeId tableHeader::GetInstanceTypeId() const
{
	return GetTypeId ();
}

uint32_t tableHeader::GetSerializedSize() const
{
	//std::cout<<"table header get size"<<std::endl;
	uint32_t size=0;
	size += sizeof(m_sourceId);
	size += sizeof(m_signature);
	Ptr<pit::nrndn::EntryNrImpl> tempPitEntry;
	//m_pitContainer.size():
	size += sizeof(uint32_t);
	for(uint32_t i = 0; i<m_pitContainer.size(); ++i)
	{
		tempPitEntry =  DynamicCast<ndn::pit::nrndn::EntryNrImpl>(m_pitContainer[i]);
		//m_interest_name.size():
		size += sizeof(uint32_t);
		size += tempPitEntry->getEntryName().size();

		//m_incomingnbs.size():
		size += sizeof(uint32_t);
		std::unordered_set<std::string>::const_iterator it;
		for(it = (tempPitEntry->getIncomingnbs()).begin() ; it != tempPitEntry->getIncomingnbs().end(); ++it)
		{
			size += sizeof(uint32_t);
			size +=(*it).size();
		}
	}

	Ptr<fib::nrndn::EntryNrImpl> tempFibEntry;
	//m_fibContainer.size():
	size += sizeof(uint32_t);
	for(uint32_t i = 0; i<m_fibContainer.size(); ++i)
	{
		 tempFibEntry =DynamicCast<ndn::fib::nrndn::EntryNrImpl>( m_fibContainer[i]);
		//m_interest_name.size():
		size += sizeof(uint32_t);
		size +=  tempFibEntry->getEntryName().size();

		//m_incomingnbs.size():
		size += sizeof(uint32_t);
		std::unordered_map<std::string, uint32_t >::const_iterator it;
		for(it = ( tempFibEntry->getIncomingnbs()).begin(); it !=  tempFibEntry->getIncomingnbs().end(); ++it)
		{
			size += sizeof(uint32_t);
			size +=it->first.size();
			size += sizeof(uint32_t);
		}
	}
	//std::cout<<"get deserialize size:"<<size<<std::endl;
	return size;
}

void tableHeader::Serialize(Buffer::Iterator start) const
{
	//std::cout<<"serialize:"<<std::endl;
	Buffer::Iterator& i = start;
	i.WriteHtonU32(m_sourceId);
	i.WriteHtonU32(m_signature);
	//std::cout<<"serialize:  source id:"<<m_sourceId<<" m_pitContainer.size():"<<m_pitContainer.size()<<"  m_fibContainer.size():"<<m_fibContainer.size()<<std::endl;
	Ptr<pit::nrndn::EntryNrImpl> tempPitEntry;
	i.WriteHtonU32(m_pitContainer.size());
	for(uint32_t it = 0; it<m_pitContainer.size(); ++it)
	{
		tempPitEntry =DynamicCast<ndn::pit::nrndn::EntryNrImpl>( m_pitContainer[it]);
		i.WriteHtonU32(tempPitEntry->getEntryName().size());
		for(uint32_t j = 0; j<tempPitEntry->getEntryName().size(); ++j)
				i.Write((uint8_t*)&((tempPitEntry->getEntryName())[j]),sizeof(char));

		i.WriteHtonU32(tempPitEntry->getIncomingnbs().size());
		std::unordered_set<std::string>::const_iterator j;
		for(j = (tempPitEntry->getIncomingnbs()).begin(); j != tempPitEntry->getIncomingnbs().end(); ++j)
		{
			i.WriteHtonU32((*j).size());
			for(uint32_t k = 0; k<(*j).size(); ++k)
					i.Write((uint8_t*)&((*j)[k]),sizeof(char));
		}
	}

	Ptr<fib::nrndn::EntryNrImpl> tempFibEntry;
	i.WriteHtonU32(m_fibContainer.size());
	for(uint32_t it = 0; it<m_fibContainer.size(); ++it)
	{
		tempFibEntry =DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fibContainer[it]);
		i.WriteHtonU32(tempFibEntry->getEntryName().size());
		//std::cout<<"name size:"<<tempFibEntry->getEntryName().size()<<std::endl;
		//std::cout<<tempFibEntry->getEntryName()<<std::endl;
		for(uint32_t j = 0; j<tempFibEntry->getEntryName().size(); ++j)
				i.Write((uint8_t*)&((tempFibEntry->getEntryName())[j]),sizeof(char));

		i.WriteHtonU32(tempFibEntry->getIncomingnbs().size());
		std::unordered_map<std::string, uint32_t >::const_iterator j;
		for(j = (tempFibEntry->getIncomingnbs()).begin(); j != tempFibEntry->getIncomingnbs().end(); ++j)
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
	//std::cout<<"deserialize:"<<std::endl;
	Buffer::Iterator i = start;
	m_sourceId	=	i.ReadNtohU32();
	m_signature =	i.ReadNtohU32();
	m_pitContainer.clear();

	uint32_t size  = i.ReadNtohU32();
	//std::cout<<"deserialize:  source id:"<<m_sourceId<<" m_pitContainer.size():"<<size;
	for (uint32_t j = 0; j < size; j++)
	{
		char tempchar;
		std::string tempstring;
		uint32_t namesize =  i.ReadNtohU32();
		for(uint32_t k = 0; k < namesize; ++k)
		{
			i.Read((uint8_t*)&(tempchar),sizeof(char));
			tempstring += tempchar;
		}
		Time interval(300);
		Ptr<Interest> interest = Create<Interest> (Create<Packet>(1024));
		Ptr<Name> interestName = Create<Name> (tempstring);
		interest->SetName(interestName);
		Ptr<fib::Entry> fibEntry=ns3::Create<fib::Entry>(Ptr<Fib>(0),Ptr<Name>(0));
		Ptr<pit::nrndn::EntryNrImpl>  temp = ns3::Create<pit::nrndn::EntryNrImpl>(*m_pit ,interest,fibEntry);
		temp->setInterestName(tempstring);

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
		temp->setNb(tempnb);
		m_pitContainer.push_back(DynamicCast<ndn::pit::Entry>(temp));
	}

	m_fibContainer.clear();
	size = i.ReadNtohU32();
	//std::cout<<"  m_fibContainer.size():"<<size<<std::endl;
	for (uint32_t j = 0; j < size; j++)
	{
		char tempchar;
		std::string tempstring;
		uint32_t namesize =  i.ReadNtohU32();
		//std::cout<<"name size:"<<namesize<<std::endl;
		for(uint32_t k = 0; k < namesize; ++k)
		{
			i.Read((uint8_t*)&(tempchar),sizeof(char));
			tempstring += tempchar;
		}
		Ptr<Name>name = Create<Name> (tempstring);
		Time interval(300);
		Ptr<fib::nrndn::EntryNrImpl> temp = ns3::Create<fib::nrndn::EntryNrImpl>(m_fib,name, interval);
		temp->setDataName(tempstring);

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
		temp->setNb(tempnb);
		m_fibContainer.push_back(DynamicCast<ndn::fib::Entry>(temp));
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
