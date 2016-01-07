/*
 * ndn-navigation-route-heuristic-forwarding.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#include "ndn-navigation-route-heuristic-forwarding.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "nrndn-Header.h"
#include "nrndn-Neighbors.h"
#include "ndn-pit-entry-nrimpl.h"
#include "nrUtils.h"
#include "ndn-packet-type-tag.h"

#include "ns3/core-module.h"
#include "ns3/ptr.h"
#include "ns3/ndn-interest.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"

#include <algorithm>    // std::find

NS_LOG_COMPONENT_DEFINE ("ndn.fw.NavigationRouteHeuristic");

namespace ns3
{
namespace ndn
{
namespace fw
{
namespace nrndn
{

NS_OBJECT_ENSURE_REGISTERED (NavigationRouteHeuristic);

TypeId NavigationRouteHeuristic::GetTypeId(void)
{
	  static TypeId tid = TypeId ("ns3::ndn::fw::nrndn::NavigationRouteHeuristic")
	    .SetGroupName ("Ndn")
	    .SetParent<GreenYellowRed> ()
	    .AddConstructor<NavigationRouteHeuristic>()
	    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
	            TimeValue (Seconds (1)),
	            MakeTimeAccessor (&NavigationRouteHeuristic::HelloInterval),
	            MakeTimeChecker ())
	     .AddAttribute ("AllowedHelloLoss", "Number of hello messages which may be loss for valid link.",
	            UintegerValue (2),
	            MakeUintegerAccessor (&NavigationRouteHeuristic::AllowedHelloLoss),
	            MakeUintegerChecker<uint32_t> ())

	   	 .AddAttribute ("gap", "the time gap between interested nodes and disinterested nodes for sending a data packet.",
	   	        UintegerValue (20),
	   	        MakeUintegerAccessor (&NavigationRouteHeuristic::m_gap),
	   	        MakeUintegerChecker<uint32_t> ())
//		.AddAttribute ("CacheSize", "The size of the cache which records the packet sent, use LRU scheme",
//				UintegerValue (6000),
//				MakeUintegerAccessor (&NavigationRouteHeuristic::SetCacheSize,
//									  &NavigationRouteHeuristic::GetCacheSize),
//				MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("UniformRv", "Access to the underlying UniformRandomVariable",
        		 StringValue ("ns3::UniformRandomVariable"),
        		 MakePointerAccessor (&NavigationRouteHeuristic::m_uniformRandomVariable),
        		 MakePointerChecker<UniformRandomVariable> ())
        .AddAttribute ("HelloLogEnable", "the switch which can turn on the log on Functions about hello messages",
        		 BooleanValue (true),
        		 MakeBooleanAccessor (&NavigationRouteHeuristic::m_HelloLogEnable),
        		 MakeBooleanChecker())
        .AddAttribute ("NoFwStop", "When the PIT covers the nodes behind, no broadcast stop message",
        		 BooleanValue (false),
        		 MakeBooleanAccessor (&NavigationRouteHeuristic::NoFwStop),
        		 MakeBooleanChecker())
		.AddAttribute ("TTLMax", "This value indicate that when a data is received by disinterested node, the max hop count it should be forwarded",
		         UintegerValue (3),
		         MakeUintegerAccessor (&NavigationRouteHeuristic::m_TTLMax),
		         MakeUintegerChecker<uint32_t> ())
	    ;
	  return tid;
}

NavigationRouteHeuristic::NavigationRouteHeuristic():
	HelloInterval (Seconds (1)),
	AllowedHelloLoss (2),
	m_htimer (Timer::CANCEL_ON_DESTROY),
	m_timeSlot(Seconds (0.05)),
	m_CacheSize(5000),// Cache size can not change. Because if you change the size, the m_interestNonceSeen and m_dataNonceSeen also need to change. It is really unnecessary
	m_interestNonceSeen(m_CacheSize),
	m_dataSignatureSeen(m_CacheSize),
	m_nb (HelloInterval),
	m_running(false),
	m_runningCounter(0),
	m_HelloLogEnable(true),
	m_gap(10),
	m_TTLMax(3),
	NoFwStop(false)
{

	m_htimer.SetFunction (&NavigationRouteHeuristic::HelloTimerExpire, this);
	m_nb.SetCallback (MakeCallback (&NavigationRouteHeuristic::FindBreaksLinkToNextHop, this));

}

NavigationRouteHeuristic::~NavigationRouteHeuristic ()
{

}

void NavigationRouteHeuristic::Start()
{
	NS_LOG_FUNCTION (this);
	if(!m_runningCounter)
	{
		m_running = true;
		m_offset = MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100));
		m_htimer.Schedule(m_offset);
		m_nb.ScheduleTimer();
	}
	m_runningCounter++;
}

void NavigationRouteHeuristic::Stop()
{
	NS_LOG_FUNCTION (this);
	if(m_runningCounter)
		m_runningCounter--;
	else
		return;

	if(m_runningCounter)
		return;
	m_running = false;
	m_htimer.Cancel();
	m_nb.CancelTimer();
}

void NavigationRouteHeuristic::WillSatisfyPendingInterest(
		Ptr<Face> inFace, Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

bool NavigationRouteHeuristic::DoPropagateInterest(
		Ptr<Face> inFace, Ptr<const Interest> interest,
		Ptr<pit::Entry> pitEntry)
{
	  NS_LOG_FUNCTION (this);
	  NS_LOG_UNCOND(this <<" is in unused function");
	  NS_ASSERT_MSG (m_pit != 0, "PIT should be aggregated with forwarding strategy");

	  return true;
}

void NavigationRouteHeuristic::WillEraseTimedOutPendingInterest(
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void NavigationRouteHeuristic::AddFace(Ptr<Face> face)
{
	//every time face is added to NDN stack?
	NS_LOG_FUNCTION(this);
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" add application face "<<face->GetId());
		m_inFaceList.push_back(face);
	}
	else
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" add NOT application face "<<face->GetId());
		m_outFaceList.push_back(face);
	}
}

void NavigationRouteHeuristic::RemoveFace(Ptr<Face> face)
{
	NS_LOG_FUNCTION(this);
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" remove application face "<<face->GetId());
		m_inFaceList.erase(find(m_inFaceList.begin(),m_inFaceList.end(),face));
	}
	else
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" remove NOT application face "<<face->GetId());
		m_outFaceList.erase(find(m_outFaceList.begin(),m_outFaceList.end(),face));
	}
}
void NavigationRouteHeuristic::DidReceiveValidNack(
		Ptr<Face> incomingFace, uint32_t nackCode, Ptr<const Interest> nack,
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void NavigationRouteHeuristic::OnInterest(Ptr<Face> face,
		Ptr<Interest> interest)
{
	//NS_LOG_UNCOND("Here is NavigationRouteHeuristic dealing with OnInterest");
	//NS_LOG_FUNCTION (this);
	if(!m_running) return;

	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Get interest packet from APPLICATION");
		// This is the source interest from the upper node application (eg, nrConsumer) of itself
		// 1.Set the payload
		interest->SetPayload(GetNrPayload(HeaderHelper::INTEREST_NDNSIM,interest->GetPayload()));

		// 2. record the Interest Packet
		m_interestNonceSeen.Put(interest->GetNonce(),true);

		// 3. Then forward the interest packet directly
		Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(0,100)),
				&NavigationRouteHeuristic::SendInterestPacket,this,interest);
		return;
	}

	if(HELLO_MESSAGE==interest->GetScope())
	{
		ProcessHello(interest);
		return;
	}

	//If the interest packet has already been sent, do not proceed the packet
	if(m_interestNonceSeen.Get(interest->GetNonce()))
	{
		NS_LOG_DEBUG("The interest packet has already been sent, do not proceed the packet of "<<interest->GetNonce());
		return;
	}

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	uint32_t nodeId;
	uint32_t seq;
	ndn::nrndn::nrHeader nrheader;
	nrPayload->PeekHeader( nrheader);
	nodeId=nrheader.getSourceId();
	seq=interest->GetNonce();
	const std::vector<uint32_t>& pri=nrheader.getPriorityList();

	//Deal with the stop message first
	if(Interest::NACK_LOOP==interest->GetNack())
	{
		ExpireInterestPacketTimer(nodeId,seq);
		return;
	}

	//If it is not a stop message, prepare to forward:
	pair<bool, double> msgdirection =
			packetFromDirection(interest);
	if(!msgdirection.first || // from other direction
			msgdirection.second > 0)// or from front
	{
		NS_LOG_DEBUG("Get interest packet from front or other direction");
		if(!isDuplicatedInterest(nodeId,seq))// Is new packet
		{
			NS_LOG_DEBUG("Get interest packet from front or other direction and it is new packet");
			DropInterestePacket(interest);
		}
		else // Is old packet
		{
			std::cout<<"duplicate packet"<<endl<<endl;
			NS_LOG_DEBUG("Get interest packet from front or other direction and it is old packet");
			ExpireInterestPacketTimer(nodeId,seq);
		}
	}
	else// it is from nodes behind
	{
		NS_LOG_DEBUG("Get interest packet from nodes behind");
		const vector<string> remoteRoute=
							ExtractRouteFromName(interest->GetName());

		// Update the PIT here
		m_nrpit->UpdatePit(remoteRoute, nodeId);
		// Update finish

		//evaluate whether receiver's id is in sender's priority list
		bool idIsInPriorityList;
		vector<uint32_t>::const_iterator idit;
		idit = find(pri.begin(), pri.end(), m_node->GetId());
		idIsInPriorityList = (idit != pri.end());

		//evaluate end

		if (idIsInPriorityList)
		{
			NS_LOG_DEBUG("Node id is in PriorityList");

			bool IsPitCoverTheRestOfRoute=PitCoverTheRestOfRoute(remoteRoute);

			NS_LOG_DEBUG("IsPitCoverTheRestOfRoute?"<<IsPitCoverTheRestOfRoute);
			if(NoFwStop)
				IsPitCoverTheRestOfRoute = false;

			if (IsPitCoverTheRestOfRoute)
			{
				BroadcastStopMessage(interest);
				return;
			}
			else
			{
				//Start a timer and wait
				double index = distance(pri.begin(), idit);
				double random = m_uniformRandomVariable->GetInteger(0, 20);
				Time sendInterval(MilliSeconds(random) + index * m_timeSlot);
				m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
						&NavigationRouteHeuristic::ForwardInterestPacket, this,
						interest);
			}
		}
		else
		{
			NS_LOG_DEBUG("Node id is not in PriorityList");
			DropInterestePacket(interest);
		}
	}
}

void NavigationRouteHeuristic::OnData(Ptr<Face> face, Ptr<Data> data)
{
	NS_LOG_FUNCTION (this);
	if(!m_running) return;
	if(Face::APPLICATION & face->GetFlags())
	{
		NS_LOG_DEBUG("Get data packet from APPLICATION");

		// 2. record the Data Packet(only record the forwarded packet)
		m_dataSignatureSeen.Put(data->GetSignature(),true);

		// 3. Then forward the data packet directly
		Simulator::Schedule(
				MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
				&NavigationRouteHeuristic::SendDataPacket, this, data);

		// 4. Although it is from itself, include into the receive record
		///////////NotifyUpperLayer(data);
		return;
	}

	//If the data packet has already been sent, do not proceed the packet
	if(m_dataSignatureSeen.Get(data->GetSignature()))
	{
		NS_LOG_DEBUG("The Data packet has already been sent, do not proceed the packet of "<<data->GetSignature());
		return;
	}

	Ptr<const Packet> nrPayload	= data->GetPayload();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload	->PeekPacketTag(hopCountTag);
	ndn::nrndn::PacketTypeTag packetTypeTag;
	nrPayload	->PeekPacketTag(packetTypeTag);
	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();

	uint32_t signature=data->GetSignature();
	std::string currentLane = nrheader.getCurrentLane();
	std:string preLane = nrheader.getPreLane();
	std::vector<std::string> laneList = nrheader.getLaneList();

	if(RESOURCE_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			/*
			if(no fib entry)    //没有FIB表项
			{
						//建立FIB表项
			}
			else
			{
						//更新FIB表项
			}
			*/
			double disX = ( x - m_sensor->getX()) > 0 ? (x - m_sensor->getX()) : (m_sensor->getX() - x);
			double disY = ( y - m_sensor->getY()) > 0 ? (y - m_sensor->getY()) : (m_sensor->getY() - y);
			double distance = disX + disY;
			cout<<"distance:"<<distance<<endl;
			double interval = (500 - distance) * 10;
			Time sendInterval = (MilliSeconds(interval) + m_gap * m_timeSlot);
			m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardResourcePacket, this,data);
		}//end if(!isDuplicatedData(nodeId,signature))
		else   //重复包
		{
			if(m_sensor->getLane() == currentLane)
			{
				ExpireDataPacketTimer(nodeId,signature);
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
			else
			{
				if(m_sensor->getLane() == preLane)
				{
					ExpireDataPacketTimer(nodeId,signature);
					m_dataSignatureSeen.Put(data->GetSignature(),true);
					return;
				}
			}
		}//end duplicate packet
	}//end if (RESOURCE_PACKET == packetTypeTag.Get())
	else if (DATA_PACKET == packetTypeTag.Get())
	{

	}
	else if(CONFIRM_PACKET == packetTypeTag.Get())
	{

	}
	else if(TABLE_PACKET == packetTypeTag.Get())
	{

	}
	ToContentStore(data);

	Ptr<pit::Entry> Will = WillInterestedData(data);
	Ptr<pit::nrndn::EntryNrImpl> entry = DynamicCast<pit::nrndn::EntryNrImpl>(Will);

	return;
}

pair<bool, double>
NavigationRouteHeuristic::packetFromDirection(Ptr<Interest> interest)
{
	NS_LOG_FUNCTION (this);
	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::nrHeader nrheader;
	nrPayload->PeekHeader( nrheader);
	const vector<string> route	= ExtractRouteFromName(interest->GetName());

	pair<bool, double> result =
			m_sensor->getDistanceWith(nrheader.getX(),nrheader.getY(),route);

	return result;
}

bool NavigationRouteHeuristic::isDuplicatedInterest(
		uint32_t id, uint32_t nonce)
{
	NS_LOG_FUNCTION (this);
	if(!m_sendingInterestEvent.count(id))
		return false;
	else
		return m_sendingInterestEvent[id].count(nonce);
}

void NavigationRouteHeuristic::ExpireInterestPacketTimer(uint32_t nodeId,uint32_t seq)
{
	NS_LOG_FUNCTION (this<< "ExpireInterestPacketTimer id"<<nodeId<<"sequence"<<seq);
	//1. Find the waiting timer
	EventId& eventid = m_sendingInterestEvent[nodeId][seq];
	
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void NavigationRouteHeuristic::BroadcastStopMessage(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this<<" broadcast a stop message of "<<src->GetName().toUri());
	//1. copy the interest packet
	Ptr<Interest> interest = Create<Interest> (*src);

	//2. set the nack type
	interest->SetNack(Interest::NACK_LOOP);

	//3.Remove the useless payload, save the bandwidth
	Ptr<const Packet> nrPayload=src->GetPayload();
	ndn::nrndn::nrHeader srcheader,dstheader;
	nrPayload->PeekHeader( srcheader);
	dstheader.setSourceId(srcheader.getSourceId());
	Ptr<Packet> newPayload	= Create<Packet> ();
	newPayload->AddHeader(dstheader);
	interest->SetPayload(newPayload);

	//4. send the payload
	Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(0,10)),
					&NavigationRouteHeuristic::SendInterestPacket,this,interest);

}

void NavigationRouteHeuristic::ForwardResourcePacket(Ptr<Data> src)
{
	if(!m_running) return;

	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	std::string currentlane = m_sensor->getLane();
	std::string prelane;
	if(currentlane == nrheader.getCurrentLane() )
		prelane = nrheader.getPreLane();
	else
		prelane = nrheader.getCurrentLane();

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(prelane);
	nrPayload->AddHeader(nrheader);

	// 	2.2 setup FwHopCountTag
	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	hopCountTag. Increment();
	if(hopCountTag.Get() > 15)
	{
		m_dataSignatureSeen.Put(src->GetSignature(),true);
		return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	m_dataSignatureSeen.Put(src->GetSignature(),true);
	// 3. Send the data Packet. Already wait, so no schedule
	SendDataPacket(data);
}

void NavigationRouteHeuristic::ForwardDataPacket(Ptr<Data> src,std::vector<uint32_t> newPriorityList,bool IsClearhopCountTag)
{
	if(!m_running) return;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::ForwardDataPacket");
	NS_LOG_FUNCTION (this);
	uint32_t sourceId=0;
	uint32_t signature=0;
	// 1. record the Data Packet(only record the forwarded packet)
	m_dataSignatureSeen.Put(src->GetSignature(),true);

	// 2. prepare the data
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	sourceId = nrheader.getSourceId();
	signature = src->GetSignature();

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setPriorityList(newPriorityList);
	nrPayload->AddHeader(nrheader);

	// 	2.2 setup FwHopCountTag
	//FwHopCountTag hopCountTag;
	//if (!IsClearhopCountTag)
	//	nrPayload->PeekPacketTag( hopCountTag);
//	newPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	// 3. Send the data Packet. Already wait, so no schedule
	SendDataPacket(data);

	// 4. record the forward times
	//////ndn::nrndn::nrUtils::IncreaseForwardCounter(sourceId,signature);
}

void NavigationRouteHeuristic::ForwardInterestPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	uint32_t sourceId=0;
	uint32_t nonce=0;

	// 1. record the Interest Packet(only record the forwarded packet)
	m_interestNonceSeen.Put(src->GetNonce(),true);

	// 2. prepare the interest
	Ptr<const Packet> nrPayload=src->GetPayload();
	ndn::nrndn::nrHeader nrheader;
	nrPayload->PeekHeader( nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	const vector<string> route	=
			ExtractRouteFromName(src->GetName());
	const std::vector<uint32_t> priorityList=GetPriorityList(route);
	sourceId=nrheader.getSourceId();
	nonce   =src->GetNonce();
	// source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setPriorityList(priorityList);

	Ptr<Packet> newPayload	= Create<Packet> ();
	newPayload->AddHeader(nrheader);

	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(newPayload);

	// 3. Send the interest Packet. Already wait, so no schedule
	SendInterestPacket(interest);

	// 4. record the forward times
	ndn::nrndn::nrUtils::IncreaseInterestForwardCounter(sourceId,nonce);
}

bool NavigationRouteHeuristic::PitCoverTheRestOfRoute(
		const vector<string>& remoteRoute)
{
	NS_LOG_FUNCTION (this);
	const vector<string>& localRoute =m_sensor->getNavigationRoute();
	const string& localLane=m_sensor->getLane();
	vector<string>::const_iterator localRouteIt;
	vector<string>::const_iterator remoteRouteIt;
	localRouteIt  = std::find( localRoute.begin(), localRoute.end(), localLane);
	remoteRouteIt = std::find(remoteRoute.begin(), remoteRoute.end(),localLane);

	//The route is in reverse order in Name, example:R1-R2-R3, the name is /R3/R2/R1
	while (localRouteIt != localRoute.end()
			&& remoteRouteIt != remoteRoute.end())
	{
		if (*localRouteIt == *remoteRouteIt)
		{
			++localRouteIt;
			++remoteRouteIt;
		}
		else
			break;
	}
	return (remoteRouteIt == remoteRoute.end());
}

void NavigationRouteHeuristic::DoInitialize(void)
{
	super::DoInitialize();
}

void NavigationRouteHeuristic::DropPacket()
{
	NS_LOG_DEBUG ("Drop Packet");
}

void NavigationRouteHeuristic::DropDataPacket(Ptr<Data> data)
{
	NS_LOG_DEBUG ("Drop data Packet");
	/*
	 * @Date 2015-03-17 For statistics reason, count the disinterested data
	 * */
	//Choice 1:
	NotifyUpperLayer(data);

	//Choice 2:
	//DropPacket();
}

void NavigationRouteHeuristic::DropInterestePacket(Ptr<Interest> interest)
{
	NS_LOG_DEBUG ("Drop interest Packet");
	DropPacket();
}

void NavigationRouteHeuristic::SendInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;

	if(HELLO_MESSAGE!=interest->GetScope()||m_HelloLogEnable)
		NS_LOG_FUNCTION (this);

	//    if the node has multiple out Netdevice face, send the interest package to them all
	//    makde sure this is a NetDeviceFace!!!!!!!!!!!1
	vector<Ptr<Face> >::iterator fit;
	for(fit=m_outFaceList.begin();fit!=m_outFaceList.end();++fit)
	{
		(*fit)->SendInterest(interest);
		//////ndn::nrndn::nrUtils::AggrateInterestPacketSize(interest);
	}
}


void NavigationRouteHeuristic::NotifyNewAggregate()
{

  if (m_sensor == 0)
  {
	  m_sensor = GetObject<ndn::nrndn::NodeSensor> ();
   }

  if (m_nrpit == 0)
  {
	  Ptr<Pit> pit=GetObject<Pit>();
	  if(pit)
		  m_nrpit = DynamicCast<pit::nrndn::NrPitImpl>(pit);
  }
  if(m_node==0)
  {
	  m_node=GetObject<Node>();
  }

  super::NotifyNewAggregate ();
}


void
NavigationRouteHeuristic::HelloTimerExpire ()
{
	if(!m_running) return;

	if (m_HelloLogEnable)
		NS_LOG_FUNCTION(this);
	SendHello();

	m_htimer.Cancel();
	Time base(HelloInterval - m_offset);
	m_offset = MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100));
	m_htimer.Schedule(base + m_offset);
}

void
NavigationRouteHeuristic::FindBreaksLinkToNextHop(uint32_t BreakLinkNodeId)
{
	 NS_LOG_FUNCTION (this);
}
/*
void NavigationRouteHeuristic::SetCacheSize(uint32_t cacheSize)
{
	NS_LOG_FUNCTION(this);
	NS_LOG_INFO("Change cache size from "<<m_CacheSize<<" to "<<cacheSize);
	m_CacheSize = cacheSize;
	m_interestNonceSeen=ndn::nrndn::cache::LRUCache<uint32_t,bool>(m_CacheSize);
	m_dataNonceSeen=ndn::nrndn::cache::LRUCache<uint32_t,bool>(m_CacheSize);
}
*/
void
NavigationRouteHeuristic::SendHello()
{
	if(!m_running) return;

	if (m_HelloLogEnable)
		NS_LOG_FUNCTION(this);
	const double& x		= m_sensor->getX();
	const double& y		= m_sensor->getY();
	const string& LaneName=m_sensor->getLane();
	//1.setup name
	Ptr<Name> name = ns3::Create<Name>('/'+LaneName);

	//2. setup payload
	Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrHeader nrheader;
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setSourceId(m_node->GetId());
	newPayload->AddHeader(nrheader);

	//3. setup interest packet
	Ptr<Interest> interest	= Create<Interest> (newPayload);
	interest->SetScope(HELLO_MESSAGE);	// The flag indicate it is hello message
	interest->SetName(name); //interest name is lane;

	//4. send the hello message
	SendInterestPacket(interest);

}

void
NavigationRouteHeuristic::ProcessHello(Ptr<Interest> interest)
{
	if(!m_running) return;

	if(m_HelloLogEnable)
		NS_LOG_DEBUG (this << interest << "\tReceived HELLO packet from "<<interest->GetNonce());

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::nrHeader nrheader;
	nrPayload->PeekHeader(nrheader);
	//update neighbor list
	m_nb.Update(nrheader.getSourceId(),nrheader.getX(),nrheader.getY(),Time (AllowedHelloLoss * HelloInterval));
}

std::vector<uint32_t> NavigationRouteHeuristic::GetPriorityList()
{
	return GetPriorityList(m_sensor->getNavigationRoute());
}

vector<string> NavigationRouteHeuristic::ExtractRouteFromName(const Name& name)
{
	// Name is in reverse order, so reverse it again
	// eg. if the navigation route is R1-R2-R3, the name is /R3/R2/R1
	vector<string> result;
	Name::const_reverse_iterator it;
	for(it=name.rbegin();it!=name.rend();++it)
		result.push_back(it->toUri());
	return result;
}

Ptr<Packet> NavigationRouteHeuristic::GetNrPayload(HeaderHelper::Type type, Ptr<const Packet> srcPayload, const Name& dataName /*= *((Name*)NULL) */)
{
	NS_LOG_INFO("Get nr payload, type:"<<type);
	Ptr<Packet> nrPayload = Create<Packet>(*srcPayload);
	/*
	std::vector<uint32_t> priorityList;
	switch (type)
	{
		case HeaderHelper::INTEREST_NDNSIM:
		{
			priorityList = GetPriorityList();
			break;
		}
		case HeaderHelper::CONTENT_OBJECT_NDNSIM:
		{
			priorityList = GetPriorityListOfDataSource(dataName);
			if(priorityList.empty())//There is no interested nodes behind
				return Create<Packet>();
			break;
		}
		default:
		{
			NS_ASSERT_MSG(false, "unrecognize packet type");
			break;
		}
	}
	*/

	const double& x = m_sensor->getX();
	const double& y = m_sensor->getY();
	ndn::nrndn::nrndnHeader nrheader;
	nrheader.setX(x);
	nrheader.setY(y);
	nrPayload->AddHeader(nrheader);
	return nrPayload;
}

std::vector<uint32_t> NavigationRouteHeuristic::GetPriorityListOfDataSource(const Name& dataName)
{
	NS_LOG_INFO("GetPriorityListOfDataSource");
	std::vector<uint32_t> priorityList;
	std::multimap<double,uint32_t,std::greater<double> > sortInterest;
	std::multimap<double,uint32_t,std::greater<double> > sortNotInterest;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::GetPriorityListOfDataFw");
	Ptr<pit::nrndn::EntryNrImpl> entry = DynamicCast<pit::nrndn::EntryNrImpl>(m_nrpit->Find(dataName));
	NS_ASSERT_MSG(entry!=0," entry not find, NodeID:"<<m_node->GetId()<<" At time:"<<Simulator::Now().GetSeconds()
			<<" Current dataName:"<<dataName.toUri());
	const std::unordered_set<uint32_t>& interestNodes = entry->getIncomingnbs();
	const vector<string>& route  = m_sensor->getNavigationRoute();
	if(!interestNodes.empty())// There is interested nodes behind
	{
		std::unordered_map<uint32_t, Neighbors::Neighbor>::const_iterator nb;
		for(nb=m_nb.getNb().begin();nb!=m_nb.getNb().end();++nb)//O(n) complexity
		{
			std::pair<bool, double> result = m_sensor->getDistanceWith(nb->second.m_x,nb->second.m_y,route);
			if(interestNodes.count(nb->first))//O(1) complexity
			{
				if(!result.first)//from other direction, maybe from other lane behind
				{
					NS_LOG_DEBUG("At "<<Simulator::Now().GetSeconds()<<", "<<m_node->GetId()<<"'s neighbor "<<nb->first<<" do not on its route, but in its interest list.Check!!");
					sortInterest.insert(std::pair<double,uint32_t>(result.second,nb->first));
				}
				else{// from local route behind
					if(result.second < 0)
						sortInterest.insert(std::pair<double,uint32_t> ( -result.second , nb->first ) );
					else
					{
						// otherwise it is in front of route.No need to forward, simply do nothing
					}
				}
			}
			else
			{
				if(!result.first)//from other direction, maybe from other lane behind
					sortNotInterest.insert(std::pair<double,uint32_t>(result.second,nb->first));
				else
				{
					if(result.second<0)
						sortNotInterest.insert(std::pair<double,uint32_t> ( -result.second , nb->first ) );
				}
			}
		}

		std::multimap<double,uint32_t,std::greater<double> >::iterator it;
		//setp 1. push the interested nodes
		for(it = sortInterest.begin();it!=sortInterest.end();++it)
			priorityList.push_back(it->second);

		//step 2. push the not interested nodes
		for(it = sortNotInterest.begin();it!=sortNotInterest.end();++it)
			priorityList.push_back(it->second);
	}
	return priorityList;
}

void NavigationRouteHeuristic::ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature)
{
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::ExpireDataPacketTimer");
	NS_LOG_FUNCTION (this<< "ExpireDataPacketTimer id\t"<<nodeId<<"\tsignature:"<<signature);
	//1. Find the waiting timer
	EventId& eventid = m_sendingDataEvent[nodeId][signature];

	//2. cancel the timer if it is still running
	eventid.Cancel();
}

Ptr<pit::Entry>
NavigationRouteHeuristic::WillInterestedData(Ptr<const Data> data)
{
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::isInterestedData");
	return m_pit->Find(data->GetName());
}

bool NavigationRouteHeuristic::isDuplicatedData(uint32_t id, uint32_t signature)
{
	NS_LOG_FUNCTION (this);
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::isDuplicatedData");
	if(!m_sendingDataEvent.count(id))
		return false;
	else
		return m_sendingDataEvent[id].count(signature);
}

void NavigationRouteHeuristic::BroadcastStopMessage(Ptr<Data> src)
{
	if(!m_running) return;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::BroadcastStopMessage(Ptr<Data> src)");

	NS_LOG_FUNCTION (this<<" broadcast a stop message of "<<src->GetName().toUri());
	//1. copy the interest packet
	Ptr<Data> data = Create<Data> (*src);

	//2.Remove the useless payload, save the bandwidth
	Ptr<const Packet> nrPayload=src->GetPayload();
	ndn::nrndn::nrHeader srcheader,dstheader;
	nrPayload->PeekHeader( srcheader);
	dstheader.setSourceId(srcheader.getSourceId());//Stop message contains an empty priority list
	Ptr<Packet> newPayload	= Create<Packet> ();
	newPayload->AddHeader(dstheader);
	data->SetPayload(newPayload);

	//4. send the payload
	Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(0,10)),
					&NavigationRouteHeuristic::SendDataPacket,this,data);
}

std::vector<uint32_t> NavigationRouteHeuristic::GetPriorityListOfDataForwarderInterestd(
		const std::unordered_set<uint32_t>& interestNodes,
		const std::vector<uint32_t>& recPri)
{
	std::vector<uint32_t> priorityList;
	std::unordered_set<uint32_t> LookupPri=converVectorList(recPri);

	std::multimap<double,uint32_t,std::greater<double> > sortInterestFront;
	std::multimap<double,uint32_t,std::greater<double> > sortInterestBack;
	std::multimap<double,uint32_t,std::greater<double> > sortDisinterest;
	const vector<string>& route  = m_sensor->getNavigationRoute();

	NS_ASSERT (!interestNodes.empty());	// There must be interested nodes behind

	std::unordered_map<uint32_t, Neighbors::Neighbor>::const_iterator nb;
	for (nb = m_nb.getNb().begin(); nb != m_nb.getNb().end(); ++nb)	//O(n) complexity
	{
		std::pair<bool, double> result = m_sensor->getDistanceWith(
				nb->second.m_x, nb->second.m_y, route);
		if (interestNodes.count(nb->first))	//O(1) complexity
		{
			if (!result.first)//from other direction, maybe from other lane behind
			{
				sortInterestBack.insert(
						std::pair<double, uint32_t>(result.second, nb->first));
			}
			else
			{	if (result.second < 0)
				{	// from local route behind
					if(!LookupPri.count(nb->first))
						sortInterestBack.insert(
							std::pair<double, uint32_t>(-result.second,
									nb->first));
				}
				else
				{
					// otherwise it is in front of route, add to sortInterestFront
					sortInterestFront.insert(
							std::pair<double, uint32_t>(-result.second,
									nb->first));
				}
			}
		}
		else
		{
			if (!result.first)//from other direction, maybe from other lane behind
				sortDisinterest.insert(
						std::pair<double, uint32_t>(result.second, nb->first));
			else
			{
				if(result.second>0// nodes in front
						||!LookupPri.count(nb->first))//if it is not in front, need to not appear in priority list of last hop
				sortDisinterest.insert(
					std::pair<double, uint32_t>(-result.second, nb->first));
			}
		}
	}

	std::multimap<double, uint32_t, std::greater<double> >::iterator it;
	//setp 1. push the interested nodes from behind
	for (it = sortInterestBack.begin(); it != sortInterestBack.end(); ++it)
		priorityList.push_back(it->second);

	//step 2. push the disinterested nodes
	for (it = sortDisinterest.begin(); it != sortDisinterest.end(); ++it)
		priorityList.push_back(it->second);

	//step 3. push the interested nodes from front
	for (it = sortInterestFront.begin(); it != sortInterestFront.end(); ++it)
		priorityList.push_back(it->second);

	return priorityList;
}

std::vector<uint32_t> NavigationRouteHeuristic::GetPriorityListOfDataForwarderDisinterestd(const std::vector<uint32_t>& recPri)
{
	std::vector<uint32_t> priorityList;
	std::unordered_map<uint32_t, Neighbors::Neighbor>::const_iterator nb;
	std::unordered_set<uint32_t> LookupPri=converVectorList(recPri);
	const vector<string>& route  = m_sensor->getNavigationRoute();
	std::multimap<double,uint32_t,std::greater<double> > sortDisinterest;

	for (nb = m_nb.getNb().begin(); nb != m_nb.getNb().end(); ++nb)	//O(n) complexity
	{
		std::pair<bool, double> result = m_sensor->getDistanceWith(
				nb->second.m_x, nb->second.m_y, route);

		if (!result.first)	//from other direction, maybe from other lane behind
			sortDisinterest.insert(
					std::pair<double, uint32_t>(result.second, nb->first));
		else
		{
			if (result.second > 0	// nodes in front
			|| !LookupPri.count(nb->first))	//if it is not in front, need to not appear in priority list of last hop
				sortDisinterest.insert(
						std::pair<double, uint32_t>(-result.second, nb->first));
		}
	}

	std::multimap<double, uint32_t, std::greater<double> >::iterator it;
	//step 1. push the unknown interest nodes
	for (it = sortDisinterest.begin(); it != sortDisinterest.end(); ++it)
		priorityList.push_back(it->second);

	return priorityList;
}

void NavigationRouteHeuristic::SendDataPacket(Ptr<Data> data)
{
	if(!m_running) return;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::SendDataPacket");
	vector<Ptr<Face> >::iterator fit;
	for (fit = m_outFaceList.begin(); fit != m_outFaceList.end(); ++fit)
	{
		(*fit)->SendData(data);
		//////////ndn::nrndn::nrUtils::AggrateDataPacketSize(data);
	}
}

std::unordered_set<uint32_t> NavigationRouteHeuristic::converVectorList(
		const std::vector<uint32_t>& list)
{
	std::unordered_set<uint32_t> result;
	std::vector<uint32_t>::const_iterator it;
	for(it=list.begin();it!=list.end();++it)
		result.insert(*it);
	return result;
}

void NavigationRouteHeuristic::ToContentStore(Ptr<Data> data)
{
	NS_LOG_DEBUG ("To content store.(Just a trace)");
	return;
}



void NavigationRouteHeuristic::NotifyUpperLayer(Ptr<Data> data)
{
	if(!m_running) return;
	// 1. record the Data Packet received
	m_dataSignatureSeen.Put(data->GetSignature(),true);

	// 2. notify upper layer
	vector<Ptr<Face> >::iterator fit;
	for (fit = m_inFaceList.begin(); fit != m_inFaceList.end(); ++fit)
	{
		//App::OnData() will be executed,
		//including nrProducer::OnData.
		//But none of its business, just ignore
		(*fit)->SendData(data);
	}
}

std::vector<uint32_t> NavigationRouteHeuristic::GetPriorityList(
		const vector<string>& route /* = m_sensor->getNavigationRoute()*/)
{
	//NS_LOG_FUNCTION (this);
	std::vector<uint32_t> PriorityList;
	std::ostringstream str;
	str<<"PriorityList is";

	// The default order of multimap is ascending order,
	// but I need a descending order
	std::multimap<double,uint32_t,std::greater<double> > sortlist;

	// step 1. Find 1hop Neighbors In Front Of Route
	std::unordered_map<uint32_t, Neighbors::Neighbor>::const_iterator nb;
	for(nb = m_nb.getNb().begin() ; nb != m_nb.getNb().end();++nb)
	{
		std::pair<bool, double> result=
				m_sensor->getDistanceWith(nb->second.m_x,nb->second.m_y,route);
		// Be careful with the order, increasing or descending?
		if(result.first && result.second >= 0)
			sortlist.insert(std::pair<double,uint32_t>(result.second,nb->first));
	}
	// step 2. Sort By Distance Descending
	std::multimap<double,uint32_t>::iterator it;
	for(it=sortlist.begin();it!=sortlist.end();++it)
	{
		PriorityList.push_back(it->second);

		str<<'\t'<<it->second;
	}
	NS_LOG_DEBUG(str.str());

	return PriorityList;
}

} /* namespace nrndn */
} /* namespace fw */
} /* namespace ndn */
} /* namespace ns3 */


