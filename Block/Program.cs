using System;
using System.Collections.Generic;

namespace Blocks {
	/// <summary>
	/// This enum identifies the Direction of Traffic of the vehicle: Up or Down
	/// </summary>
	enum Direction { up, dn }

	/// <summary>
	/// This enum identifies whether the connection between blocks is static (Junction)
	/// or dynamic (Point)
	/// </summary>
	enum BlockConnectionType {
		Junction,
		Point
	}

	/// <summary>
	/// This enum identifies the position of the Point: Normal, Reverse, Moving or Unset (Failure)
	/// </summary>
	enum PositionState {
		Normal,
		Reverse,
		Moving,
		Unset
	}

	/// <summary>
	/// This enum identified the different states of an SDD: Free or Occupied
	/// </summary>
	enum SDDState {
		Free,
		Occupied
	}

	/// <summary>
	/// Position of the point associated with the Connection
	/// The CommandPosition method is called when a new order is sent
	/// The Update method shall be called every cycle of computation
	/// MovementDelay is the delay between command and new position reached in ms
	/// </summary>
	class Position {
		public readonly int MovementDelay;
		public PositionState State { get; set; }
		private int TempoCommand;
		private PositionState lastCommandState;

		/// <summary>
		/// Constructor with _MovementDelay in ms and default value of _State unset
		/// </summary>
		/// <param name="_MovementDelay"></param>
		/// <param name="_State"></param>
		public Position (int _MovementDelay, PositionState _State = PositionState.Unset)
		{
			State = _State;
			MovementDelay = _MovementDelay;
		}

		/// <summary>
		/// Command the point in new position by setting a delay - must be followed by Update method to 
		/// update the status up to point position is obtained
		/// </summary>
		/// <param name="commandState"></param>
		public void CommandPosition (PositionState commandState)
		{
			if (State == commandState || State == PositionState.Moving) return;
			else {
				lastCommandState = commandState;
				TempoCommand = MovementDelay;
				State = PositionState.Moving;
			}
		}

		/// <summary>
		/// Update method shall be called every cycle of execution of simulation
		/// The period of execution shall be provided as argument in ms
		/// </summary>
		public void Update (int period)

		{
			if (TempoCommand > 0) {
				TempoCommand -= period;
				if (TempoCommand <= 0) State = lastCommandState;
			}
		}

		/// <summary>
		/// Set the point to unset (fail detection / correspondence)
		/// </summary>
		public void SetFail ()
		{
			TempoCommand = 0;
			State = PositionState.Unset;
		}
	}

	/// <summary>
	/// This class represents the underlying physical detection of the vehicle
	/// </summary>
	class SDD {
		public readonly string Id;
		private readonly double TimerOccupy, TimerFree; // Timer between physical occupation/clearance and logical one, in ms
		private double TempoStatusChange;
		private SDDState NewState, State;

		public SDD (string _Id, double _TimerOccupy, double _TimerFree)
		{
			Id = _Id;
			TimerOccupy = _TimerOccupy;
			TimerFree = _TimerFree;
		}


		public void SetState (SDDState _State)
		{
			NewState = _State;
			TempoStatusChange = (NewState == SDDState.Free) ? TimerFree : TimerOccupy;
		}

		/// <summary>
		/// Update method shall be called every cycle of execution of simulation
		/// The period of execution shall be provided as argument in ms
		/// </summary>
		public void Update (int period)
		{
			if (TempoStatusChange > 0) {
				TempoStatusChange -= period;
				if (TempoStatusChange <= 0) State = NewState;
			}
		}
	}

	/// <summary>
	/// This class represents the connection between 2 or 3 blocks.
	/// It can be a static connection (between 2 blocks)
	/// or a dynamic connection (between 3 blocks)
	/// </summary>
	class Connection {
		public readonly BlockConnectionType Type;
		public readonly string Id;
		public Block TailBlock;
		public Block NormalBlock;
		public Block ReverseBlock; // This block is null if type is Junction
		public Position Position { get; set; }

		public Connection (BlockConnectionType _Type, string _Id)
		{
			Id = _Id;
			Type = _Type;
			if (Type == BlockConnectionType.Junction) ReverseBlock = null;
		}

		public Connection (Block tb, Block nb, Block rb, string _Id)
		{
			Id = _Id;
			Type = BlockConnectionType.Point;
			TailBlock = tb;
			NormalBlock = nb;
			ReverseBlock = rb;
			Position = new Position (5000);
		}

		public Connection (Block tb, Block nb, string _Id)
		{
			Id = _Id;
			Type = BlockConnectionType.Junction;
			TailBlock = tb;
			NormalBlock = nb;
			ReverseBlock = null;
		}
	}

	/// <summary>
	/// This class corresponds to a speed limitation at a KP
	/// </summary>
	class PSR {
		public double KP { get; set; }
		public double Speed { get; set; }

		public PSR (double _KP, double _Speed)
		{
			KP = _KP;
			Speed = _Speed;
		}
	}

	/// <summary>
	/// This class corresponds to a slope acceleration at a KP
	/// </summary>
	class Gradient {
		public double KP { get; set; }
		public double Slope { get; set; } // in percent 
		public Gradient (double _KP, double _Slope)
		{
			KP = _KP;
			Slope = _Slope;
		}
	}

	class Block {
		public readonly string Id;
		public Connection UpConnection { get; set; }
		public Connection DnConnection { get; set; }
		public double Length { get; set; }
		public SDD Sdd { get; set; }
		public List<PSR> ListPSR;
		public List<Gradient> ListGradient;

		public Block (string _Id, Connection UpC, Connection DnC, double _length, SDD _Sdd)
		{
			Id = _Id;
			UpConnection = UpC;
			DnConnection = DnC;
			Length = _length;
			Sdd = _Sdd;
			ListPSR = new List<PSR> ();
			ListGradient = new List<Gradient> ();
		}

		/// <summary>
		/// Returns the next Block, either Up or Dn
		/// </summary>
		/// <param name="TOD"></param>
		/// <returns> null if reach a chaining breaking, else next block in the intended direction </returns>
		public Block GetNextBlock (Direction TOD)
		{
			Connection Connection = (TOD == Direction.dn) ? DnConnection : UpConnection;
			if (Connection == null) return null;
			if (Connection.Type == BlockConnectionType.Junction) {
				if (Connection.NormalBlock == this) return Connection.TailBlock;
				else return Connection.NormalBlock;
			} else {
				if (Connection.Position.State == PositionState.Normal) {
					if (Connection.TailBlock == this) return Connection.NormalBlock;
					else return Connection.TailBlock;
				} else if (Connection.Position.State == PositionState.Reverse) {
					if (Connection.TailBlock == this) return Connection.ReverseBlock;
					else return Connection.TailBlock;
				} else return null;
			}
		}
	}

	class Train {
		private const double SimulationStep = 1e-1; // Simulation step of 100ms
		private const double MinDeltaLoc = 0.5;
		private readonly double RSEBTrigTime, ATCEBTrigTime, RSCutOffTime, T1, T2, T_ATO_anticipation;
		public readonly double Length, DistExtremityTo1stAxle, RotMass;
		private readonly double GammaEB, GammaSSP, GammaMinBrake, GammaMinMotoring, GammaMaxMotoring;
		public readonly double [,] GammaMotoring; // Max motoring capacity in m/s/s according to speed in m/s
		private readonly double MaxJerkMotoring, MaxJerkBraking; // Max jerk of train in m/s/s/s when motoring or braking
		public double Speed { get; set; }
		public double LocTail { get; set; }
		public double LocHead { get; set; }
		public Block BlockTail { get; set; }
		public Block BlockHead { get; set; }
		public double DeltaLoc { get; set; }
		public Direction DoT { get; set; }
		private double SpeedPreviousCycle, CeilingSpeed, TrainGradient, MaxSpeed;

		public Train (double _Length, double _DistExtremityTo1stAxle, double _GammaEB, double _GammaSSP,
		    double _GammaMinBrake, double _GammaMinMotoring, double [,] _GammaMotoring, double _RotMass,
		    double _MaxJerkMotoring, double _MaxJerkBraking, double MaxSpeed, double _RSEBTrigTime, double _ATCEBTrigTime,
		    double _RSCutOffTime, double _GammaMaxMotoring, double _T_ATO_anticipation)
		{
			Length = _Length;
			DistExtremityTo1stAxle = _DistExtremityTo1stAxle;
			GammaEB = _GammaEB;
			GammaSSP = _GammaSSP;
			GammaMinBrake = _GammaMinBrake;
			GammaMinMotoring = _GammaMinMotoring;
			GammaMotoring = _GammaMotoring;
			RotMass = _RotMass;
			MaxJerkMotoring = _MaxJerkMotoring;
			MaxJerkBraking = _MaxJerkBraking;
			GammaMaxMotoring = _GammaMaxMotoring;
			RSCutOffTime = _RSCutOffTime;
			ATCEBTrigTime = _ATCEBTrigTime;
			RSEBTrigTime = _RSEBTrigTime;
			T1 = (RSCutOffTime + ATCEBTrigTime);
			T2 = RSEBTrigTime;
			T_ATO_anticipation = _T_ATO_anticipation;
		}

		private (bool EBTriggered, bool NoMotoringRequest, double SpeedT1T2, double HeadKP) GetT1T2 ()
		{
			const double G = 9.81;
			double Time = 0;
			double Gamma = 0;
			double SpeedT1T2prec, SpeedT1T2 = Speed;
			double PosT1T2 = 0;
			bool EBTriggered = false;
			bool NoMotoringRequest = false;

			Train ProjectedTrain = new Train (Length, DistExtremityTo1stAxle, GammaEB, GammaSSP, GammaMinBrake, GammaMinMotoring,
			    GammaMotoring, RotMass, MaxJerkMotoring, MaxJerkBraking, MaxSpeed, RSEBTrigTime, ATCEBTrigTime, RSCutOffTime,
			    GammaMaxMotoring, T_ATO_anticipation);

			ProjectedTrain.SetTrainLocation (BlockHead, LocHead, DoT);
			double ProjectedTrainGradient = ProjectedTrain.TrainGradient;
			double dPosT1T2;

			while (Time < T1 + T2) {
				Time += SimulationStep;
				if (Time <= T1) Gamma = ProjectedTrainGradient * G / RotMass / 100 + GammaMaxMotoring;
				else Gamma = ProjectedTrainGradient * G / RotMass / 100;
				SpeedT1T2prec = SpeedT1T2;
				SpeedT1T2 += Gamma * SimulationStep;
				dPosT1T2 = 0.5 * (SpeedT1T2 + SpeedT1T2prec) * SimulationStep;
				PosT1T2 += dPosT1T2;
				ProjectedTrain.MoveTrain (dPosT1T2);
				if (GetCeilingSpeed () <= SpeedT1T2) { EBTriggered = true; }
			}
			return (EBTriggered, NoMotoringRequest, SpeedT1T2, PosT1T2);
		}

		public (bool EBTriggered, double Speed, double DeltaLocation) EvaluateDrivingCommand (double Command)
		{
			bool EBTriggered = false;
			bool NoMotoringRequest = false;
			double SpeedOutput = 0;
			double DeltaLocation = 0;
			double Time = 0;

			SpeedPreviousCycle = Speed;
			while (Time < T_ATO_anticipation) {
				Time += SimulationStep;
				///TODO: TO COMPLETE
			}
			(EBTriggered, NoMotoringRequest, SpeedOutput, DeltaLocation) = GetT1T2 ();
			if (EBTriggered) { return (EBTriggered, SpeedOutput, DeltaLocation); }

			return (EBTriggered, SpeedOutput, DeltaLocation);
		}

		/// <summary>
		/// This method computes the current effect of the slope under the Train
		/// </summary>
		/// <returns> false if there is an issue during computation, e.g. chainage breaking, true otherwise </returns>
		private bool RefreshSlope ()
		{
			Block CurrentBlock = BlockTail;
			bool ScrutationFinished = false;
			double KP1, KP2, SlopeSegmentValue;

			TrainGradient = KP1 = KP2 = SlopeSegmentValue = 0;

			while (!ScrutationFinished) {
				if (CurrentBlock.ListGradient.Count == 0) return false;
				foreach (Gradient gradient in CurrentBlock.ListGradient) {
					if (CurrentBlock == BlockTail && gradient.KP <= LocTail) {
						KP2 = LocTail;
						SlopeSegmentValue = gradient.Slope;
					} else if (CurrentBlock != BlockHead) {
						KP1 = KP2;
						KP2 = gradient.KP;
						TrainGradient += (KP2 - KP1) / Length * SlopeSegmentValue;
						SlopeSegmentValue = gradient.Slope;
					} else if (CurrentBlock == BlockHead && gradient.KP < LocHead) {
						KP1 = KP2;
						KP2 = gradient.KP;
						TrainGradient += (KP2 - KP1) / Length * SlopeSegmentValue;
						SlopeSegmentValue = gradient.Slope;
					} else if (CurrentBlock == BlockHead && gradient.KP >= LocHead) {
						KP1 = KP2;
						KP2 = LocHead;
						TrainGradient += (KP2 - KP1) / Length * SlopeSegmentValue;
						SlopeSegmentValue = gradient.Slope;
						ScrutationFinished = true;
					}
				}
				if (CurrentBlock == BlockHead) {
					KP1 = KP2;
					KP2 = LocHead;
					TrainGradient += (KP2 - KP1) / Length * SlopeSegmentValue;
					ScrutationFinished = true;
				} else {
					KP1 = KP2;
					TrainGradient += (CurrentBlock.Length - KP1) / Length * SlopeSegmentValue;
					KP1 = KP2 = 0;
				}

				CurrentBlock = CurrentBlock.GetNextBlock (DoT);
			}
			return true;
		}

		public double GetTrainGradient () { return TrainGradient; }

		public double GetCeilingSpeed () { return CeilingSpeed; }

		/// <summary>
		/// This private method goes from Train tail to Train head and records the ceiling speed for the Train
		/// </summary>
		/// <returns> false in case of error - e.g. chaining braking under the Train - true otherwise </returns>
		private bool RefreshCeilingSpeed ()
		{
			CeilingSpeed = 0;

			Block CurrentBlock = BlockTail;
			bool ScrutationFinished = false;

			while (!ScrutationFinished) {
				if (CurrentBlock.ListPSR.Count == 0) return false;
				foreach (PSR psr in CurrentBlock.ListPSR) {
					if ((CurrentBlock == BlockTail) && (psr.KP <= LocTail)) {
						CeilingSpeed = (CeilingSpeed > psr.Speed) ? CeilingSpeed : psr.Speed;
					} else if (CurrentBlock == BlockHead) {
						if (psr.KP <= LocHead) CeilingSpeed = (CeilingSpeed > psr.Speed) ? psr.Speed : CeilingSpeed;
					} else
						CeilingSpeed = (CeilingSpeed > psr.Speed) ? psr.Speed : CeilingSpeed;
				}
				if (CurrentBlock == BlockHead) ScrutationFinished = true;
				else {
					CurrentBlock = CurrentBlock.GetNextBlock (DoT);
					if (CurrentBlock == null) return false;
				}
			}
			return true;
		}

		/// <summary>
		/// Move the Train on the network according to its current DoT, by Distance (in meter)
		/// </summary>
		/// <param name="Distance"></param>
		/// <returns> false if an issue occured, e.g. chaining breaking, true otherwise </returns>
		public bool MoveTrain (double Distance)
		{
			double RemainingDistance;
			RemainingDistance = Distance;
			Block NewHeadBlock;
			double NewLoc;

			NewHeadBlock = BlockHead;
			NewLoc = LocHead;

			while (RemainingDistance > 0) {
				if (BlockHead.Length - LocHead > RemainingDistance) {
					NewLoc = NewLoc + RemainingDistance;
					RemainingDistance = 0;
				} else {
					RemainingDistance = NewHeadBlock.Length - NewLoc;
					NewLoc = 0;
					NewHeadBlock = NewHeadBlock.GetNextBlock (DoT);
					if (NewHeadBlock == null) return false;
				}
			}
			return (SetTrainLocation (NewHeadBlock, NewLoc, DoT));
		}

		/// <summary>
		/// Locate the Train according to the position of the head and the direction of traffic
		/// </summary>
		/// <param name="_BlockHead"></param>
		/// <param name="_LocHead"></param>
		/// <param name="_DoT"></param>
		/// <returns> true if the location is successful, false otherwise </returns>
		public bool SetTrainLocation (Block _BlockHead, double _LocHead, Direction _DoT)
		{
			DoT = _DoT;
			BlockHead = _BlockHead;
			LocHead = _LocHead;
			BlockTail = BlockHead;
			LocTail = LocHead;
			double RemainLength = Length;

			while (RemainLength > 0) {
				if (DoT == Direction.up) {
					if (LocTail >= RemainLength) {
						LocTail -= RemainLength;
						RemainLength = 0;
					} else {
						BlockTail = BlockTail.GetNextBlock (Direction.dn);
						if (BlockTail == null) return false;
						RemainLength -= LocTail;
						LocTail = BlockTail.Length;
					}
				} else {
					if (BlockTail.Length - LocTail >= RemainLength) {
						LocTail += RemainLength;
						RemainLength = 0;
					} else {
						RemainLength -= BlockTail.Length - LocTail;
						BlockTail = BlockTail.GetNextBlock (Direction.up);
						if (BlockTail == null) return false;
						LocTail = 0;
					}
				}
			}
			RefreshCeilingSpeed ();
			RefreshSlope ();
			return true;
		}
	}
	class Program {

		/// <summary>
		/// This is a test function to build a network 'manually' - to be removed ultimately
		/// </summary>
		/// <param name="list_Block"></param>
		/// <param name="list_SDD"></param>
		/// <param name="list_Connection"></param>
		static void InitNetwork (List<Block> list_Block, List<SDD> list_SDD, List<Connection> list_Connection)
		{
			double SDDTimerFree = 2000;
			double SDDTimerOccupy = 500;
			list_SDD.Add (new SDD ("SDD1", SDDTimerOccupy, SDDTimerFree));
			list_SDD.Add (new SDD ("SDD2", SDDTimerOccupy, SDDTimerFree));
			list_SDD.Add (new SDD ("SDD3", SDDTimerOccupy, SDDTimerFree));
			list_SDD.Add (new SDD ("SDD4", SDDTimerOccupy, SDDTimerFree));
			list_SDD.Add (new SDD ("SDD5", SDDTimerOccupy, SDDTimerFree));

			list_Block.Add (new Block ("1", null, null, 150, list_SDD [0]));
			list_Block.Add (new Block ("2", null, null, 250, list_SDD [1]));
			list_Block.Add (new Block ("3", null, null, 150, list_SDD [2]));
			list_Block.Add (new Block ("4", null, null, 25, list_SDD [2]));
			list_Block.Add (new Block ("5", null, null, 35, list_SDD [2]));
			list_Block.Add (new Block ("6", null, null, 110, list_SDD [3]));
			list_Block.Add (new Block ("10", null, null, 45, list_SDD [4]));
			list_Block.Add (new Block ("11", null, null, 55, list_SDD [4]));
			list_Block.Add (new Block ("12", null, null, 110, list_SDD [4]));

			List<Gradient> list_GradBl1 = new List<Gradient> ();
			List<Gradient> list_GradBl2 = new List<Gradient> ();
			List<Gradient> list_GradBl3 = new List<Gradient> ();
			List<Gradient> list_GradBl4 = new List<Gradient> ();
			List<Gradient> list_GradBl5 = new List<Gradient> ();
			List<Gradient> list_GradBl6 = new List<Gradient> ();
			List<Gradient> list_GradBl10 = new List<Gradient> ();
			List<Gradient> list_GradBl11 = new List<Gradient> ();
			List<Gradient> list_GradBl12 = new List<Gradient> ();

			List<PSR> list_PSRBl1 = new List<PSR> ();
			List<PSR> list_PSRBl2 = new List<PSR> ();
			List<PSR> list_PSRBl3 = new List<PSR> ();
			List<PSR> list_PSRBl4 = new List<PSR> ();
			List<PSR> list_PSRBl5 = new List<PSR> ();
			List<PSR> list_PSRBl6 = new List<PSR> ();
			List<PSR> list_PSRBl10 = new List<PSR> ();
			List<PSR> list_PSRBl11 = new List<PSR> ();
			List<PSR> list_PSRBl12 = new List<PSR> ();

			list_PSRBl1.Add (new PSR (0, 20));
			list_PSRBl2.Add (new PSR (0, 25));
			list_PSRBl2.Add (new PSR (80, 10));
			list_PSRBl3.Add (new PSR (0, 15));
			list_PSRBl4.Add (new PSR (0, 10));
			list_PSRBl5.Add (new PSR (0, 5));
			list_PSRBl6.Add (new PSR (0, 22));
			list_PSRBl10.Add (new PSR (0, 13));
			list_PSRBl11.Add (new PSR (0, 14));
			list_PSRBl12.Add (new PSR (0, 8));

			list_Block [0].ListPSR = list_PSRBl1;
			list_Block [1].ListPSR = list_PSRBl2;
			list_Block [2].ListPSR = list_PSRBl3;
			list_Block [3].ListPSR = list_PSRBl4;
			list_Block [4].ListPSR = list_PSRBl5;
			list_Block [5].ListPSR = list_PSRBl6;
			list_Block [6].ListPSR = list_PSRBl10;
			list_Block [7].ListPSR = list_PSRBl11;
			list_Block [8].ListPSR = list_PSRBl12;

			list_GradBl1.Add (new Gradient (0, 0));
			list_GradBl1.Add (new Gradient (50, -2));
			list_GradBl2.Add (new Gradient (0, 0));
			list_GradBl3.Add (new Gradient (0, 0));
			list_GradBl4.Add (new Gradient (0, -2));
			list_GradBl5.Add (new Gradient (0, 0));
			list_GradBl6.Add (new Gradient (0, 0));
			list_GradBl10.Add (new Gradient (0, 0));
			list_GradBl11.Add (new Gradient (0, 0));
			list_GradBl12.Add (new Gradient (0, 0));

			list_Block [0].ListGradient = list_GradBl1;
			list_Block [1].ListGradient = list_GradBl2;
			list_Block [2].ListGradient = list_GradBl3;
			list_Block [3].ListGradient = list_GradBl4;
			list_Block [4].ListGradient = list_GradBl5;
			list_Block [5].ListGradient = list_GradBl6;
			list_Block [6].ListGradient = list_GradBl10;
			list_Block [7].ListGradient = list_GradBl11;
			list_Block [8].ListGradient = list_GradBl12;

			list_Connection.Add (new Connection (list_Block [0], list_Block [1], "J1_2"));
			list_Connection.Add (new Connection (list_Block [1], list_Block [2], "J2_3"));
			list_Connection.Add (new Connection (list_Block [2], list_Block [3], list_Block [4], "P3_4_5"));
			list_Connection.Add (new Connection (list_Block [3], list_Block [5], "J4_6"));
			list_Connection.Add (new Connection (list_Block [4], list_Block [6], "J6_10"));
			list_Connection.Add (new Connection (list_Block [8], list_Block [6], list_Block [7], "P10_11_12"));

			list_Block [0].UpConnection = list_Connection [0];
			list_Block [1].DnConnection = list_Connection [0];
			list_Block [1].UpConnection = list_Connection [1];
			list_Block [2].DnConnection = list_Connection [1];
			list_Block [2].UpConnection = list_Connection [2];
			list_Block [3].DnConnection = list_Connection [2];
			list_Block [4].DnConnection = list_Connection [2];
			list_Block [3].UpConnection = list_Connection [3];
			list_Block [5].DnConnection = list_Connection [3];
			list_Block [4].UpConnection = list_Connection [4];
			list_Block [6].DnConnection = list_Connection [4];
			list_Block [6].UpConnection = list_Connection [5];
			list_Block [7].UpConnection = list_Connection [5];
			list_Block [8].DnConnection = list_Connection [5];

			list_Connection [2].Position.State = PositionState.Normal;
			list_Connection [5].Position.State = PositionState.Normal;
		}

		/// <summary>
		/// This function travels from an origin to a chainage break (end of line or point not in position)
		/// </summary>
		/// <param name="list_Block"></param>
		/// <param name="list_SDD"></param>
		/// <param name="list_Connection"></param>
		/// <param name="dir"></param>
		static void TravelNetwork (List<Block> list_Block, List<SDD> list_SDD, List<Connection> list_Connection, Direction dir)
		{
			Console.WriteLine ("Number of blocks created: " + list_Block.Count);
			foreach (Block Bl in list_Block) {
				Console.WriteLine ("Block " + Bl.Id + "; Length: " + Bl.Length + "; Block: " + Bl.Sdd.Id);
			}
			Block currentBlock = list_Block [5];
			Connection nextConnection;

			while (currentBlock != null) {
				Console.WriteLine ("Current Block is " + currentBlock.Id);
				if (dir == Direction.up) { nextConnection = currentBlock.UpConnection; } else nextConnection = currentBlock.DnConnection;
				if (nextConnection == null) currentBlock = null;
				else if (nextConnection.Type == BlockConnectionType.Junction) {
					Console.WriteLine ("Junction: " + nextConnection.Id);
					if (currentBlock == nextConnection.NormalBlock) currentBlock = nextConnection.TailBlock;
					else currentBlock = nextConnection.NormalBlock;
				} else {
					Console.WriteLine ("Point: " + nextConnection.Id);
					if (currentBlock == nextConnection.NormalBlock) {
						if (nextConnection.Position.State == PositionState.Normal) currentBlock = nextConnection.TailBlock;
						else currentBlock = null;
					} else if (currentBlock == nextConnection.ReverseBlock) {
						if (nextConnection.Position.State == PositionState.Reverse) currentBlock = nextConnection.TailBlock;
						else currentBlock = null;
					} else if (currentBlock == nextConnection.TailBlock) {
						if (nextConnection.Position.State == PositionState.Normal) currentBlock = nextConnection.NormalBlock;
						else if (nextConnection.Position.State == PositionState.Reverse) currentBlock = nextConnection.ReverseBlock;
						else currentBlock = null;
					}
				}
			}
		}

		static void Main (string [] args)
		{
			List<Block> list_Block = new List<Block> ();
			List<SDD> list_SDD = new List<SDD> ();
			List<Connection> list_Connection = new List<Connection> ();

			InitNetwork (list_Block, list_SDD, list_Connection);
			TravelNetwork (list_Block, list_SDD, list_Connection, Direction.dn);
			double [,] speedArray = { { 5.55, 1.3 }, { 10, 0.8 }, { 15, 0.5 }, { 25, 0.2 } };
			Train myTrain = new Train (70, 3.1, -1.3, -0.8, -0.15, 0.2, speedArray, 1.04, 0.5, 1, 25, 1, 0.4, 2.5, 1.5, 2.0);
			if (myTrain.SetTrainLocation (list_Block [5], 25, Direction.up)) {
				Console.WriteLine ("Head - Block: " + myTrain.BlockHead.Id + " KP: " + myTrain.LocHead +
				" Tail - Block: " + myTrain.BlockTail.Id + " KP: " + myTrain.LocTail);
			} else Console.WriteLine ("Error during location of the Train");
			Console.WriteLine ("The maximum speed under the Train is: " + myTrain.GetCeilingSpeed ());
			Console.WriteLine ("The gradient under the Train is: " + myTrain.GetTrainGradient ());

			(bool EBtriggered, double Speed, double DeltaLocation) = myTrain.EvaluateDrivingCommand (0d);
			Console.WriteLine ("Speed after T1T2: " + Speed + " ms-1, displacement: " + DeltaLocation + " m");

		}
	}
}
