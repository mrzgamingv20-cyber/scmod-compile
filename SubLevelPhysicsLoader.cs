// Sub-level physics mod for Survivalcraft 2.4.0.0 (API 1.9.5.1).
// Idea (like Sable "sub-levels"): a group of blocks that loses its support is lifted out of the terrain, becomes one
// rigid body (RigidBody3D) that falls / tumbles / slides with real physics, is drawn with a rotation matrix, and when it
// comes to rest it is put back into the terrain as blocks (or dropped as items where it does not fit).
//
// Triggers (v0.3): when the player digs a block, structures next to it that are no longer supported collapse.
//   - Trees: a felled tree pivots around the back edge of the stump (pin joint) and topples away from the player.
//   - Any other "structural" blocks (planks, bricks, cobblestone, glass, slabs, stairs, logs...) floating without support.
#pragma warning disable CS0672 // the obsolete OnMinerPlace/SubsystemUpdate overloads are still called by the game
using System;
using System.Collections.Generic;
using System.Reflection;
using Engine;
using Engine.Graphics;
using GameEntitySystem;
using HarmonyLib;
using TemplatesDatabase;

namespace Game;

public static class SubLevelConfig
{
	public const int MaxBlocksPerBody = 700;       // structure + leaves; bigger structures are left alone
	public const int MaxBodies = 4;                // simultaneous physics bodies
	public const int MaxLeafDistance = 3;          // leaves farther than this from a log are not part of a tree
	public const float MaxBodyAge = 40f;           // seconds; then the body is forced to settle
	public const float MaxVisibleDistance = 160f;
	public const float TreeKick = 0.25f;           // initial angular speed of a falling tree (rad/s)
	public const float TreeReleaseTilt = 0.8f;     // radians; the hinge is released after this much tilt
	public const bool RestoreBlocks = true;        // false = always drop items instead of placing blocks back
}

// Airship tuning (Create Aeronautics style). Block roles use vanilla blocks:
//   helm   = Crafting Table (placing it next to a structure assembles the ship; also the pilot seat)
//   balloon= Canvas block (lift)       engine = Furnace (thrust)
public static class ShipConfig
{
	public const float RequiredEnvelopeFraction = 0.30f; // share of balloon blocks that gives exactly hover lift
	public const float HoverFactor = 0.98f;              // lift / weight with no input (sinks very slowly)
	public const float AscendBonus = 0.45f;              // extra lift while Jump is held
	public const float VerticalDrag = 0.9f;              // damping of vertical speed (1/s)
	public const float RequiredEngineFraction = 0.06f;   // share of engine blocks that gives ThrustAccelAtRequired
	public const float ThrustAccelAtRequired = 4f;       // m/s^2
	public const float MaxThrustAccel = 8f;
	public const float YawRate = 0.9f;                   // rad/s at full stick
	public const float YawResponse = 2.5f;
	public const float RightingAccel = 5f;               // rad/s^2 that keeps the ship level
	public const float TiltDamping = 2.5f;
	public const float LinearDamping = 0.3f;             // air drag (1/s): top speed = accel / this
	public const float AngularDamping = 0.6f;
	public const float DismountGrace = 0.8f;             // seconds after assembly before dismount input counts
}

// Decides which blocks form a structure and whether it is anchored.
public static class StructureScanner
{
	static readonly Point3[] Neighbors6 =
	{
		new Point3(1, 0, 0), new Point3(-1, 0, 0), new Point3(0, 1, 0),
		new Point3(0, -1, 0), new Point3(0, 0, 1), new Point3(0, 0, -1)
	};

	static Block BlockOf(int value)
	{
		return BlocksManager.Blocks[Terrain.ExtractContents(value)];
	}

	public static bool IsLog(int value) { return BlockOf(value) is WoodBlock; }

	public static bool IsLeaves(int value) { return BlockOf(value) is LeavesBlock; }

	// Blocks that players build with (and trees). Natural terrain (dirt, grass, stone, sand, ores...) is NOT structural,
	// it counts as ground that supports structures.
	public static bool IsStructural(int value)
	{
		if (Terrain.ExtractContents(value) == 0)
		{
			return false;
		}
		Block b = BlockOf(value);
		return b is WoodBlock || b is LeavesBlock || b is PlanksBlock || b is BrickBlock || b is BricksBlock
			|| b is StoneBrickBlock || b is CobblestoneBlock || b is GlassBlock || b is FramedGlassBlock
			|| b is PaintedCubeBlock || b is SlabBlock || b is StairsBlock || b is FenceBlock || b is WindowBlock
			|| b is IronBlock || b is CopperBlock || b is DiamondBlock
			|| b is CanvasBlock || b is FurnaceBlock || b is LitFurnaceBlock || b is CraftingTableBlock;
	}

	public static bool IsSolid(int value)
	{
		int c = Terrain.ExtractContents(value);
		return c != 0 && BlocksManager.Blocks[c].IsCollidable_(value);
	}

	// Collects the structure containing 'seed' (not a leaf block). Returns false if it is too big.
	// cells: non-leaf blocks first (nonLeafCount of them), then leaves. anchored = supported by the ground.
	public static bool Scan(Terrain terrain, Point3 seed, HashSet<Point3> claimed,
		out List<Point3> cells, out int nonLeafCount, out bool treeLike, out bool anchored)
	{
		cells = new List<Point3>();
		nonLeafCount = 0;
		treeLike = false;
		anchored = false;
		bool supportedBelow = false;
		bool supportedSide = false;
		int logs = 0;
		bool tooBig = false;

		Queue<Point3> queue = new Queue<Point3>();
		claimed.Add(seed);
		queue.Enqueue(seed);
		while (queue.Count > 0)
		{
			Point3 p = queue.Dequeue();
			cells.Add(p);
			if (cells.Count > SubLevelConfig.MaxBlocksPerBody)
			{
				tooBig = true;
				break;
			}
			int v = terrain.GetCellValue(p.X, p.Y, p.Z);
			if (IsLog(v))
			{
				logs++;
			}
			for (int i = 0; i < 6; i++)
			{
				Point3 n = new Point3(p.X + Neighbors6[i].X, p.Y + Neighbors6[i].Y, p.Z + Neighbors6[i].Z);
				int nv = terrain.GetCellValue(n.X, n.Y, n.Z);
				if (IsSolid(nv) && !IsStructural(nv))
				{
					if (Neighbors6[i].Y == -1)
					{
						supportedBelow = true;
					}
					else
					{
						supportedSide = true;
					}
				}
			}
			for (int dx = -1; dx <= 1; dx++)
			{
				for (int dy = -1; dy <= 1; dy++)
				{
					for (int dz = -1; dz <= 1; dz++)
					{
						if (dx == 0 && dy == 0 && dz == 0)
						{
							continue;
						}
						Point3 n = new Point3(p.X + dx, p.Y + dy, p.Z + dz);
						if (n.Y < 0 || n.Y > 255 || claimed.Contains(n))
						{
							continue;
						}
						int nv = terrain.GetCellValue(n.X, n.Y, n.Z);
						if (IsStructural(nv) && !IsLeaves(nv))
						{
							claimed.Add(n);
							queue.Enqueue(n);
						}
					}
				}
			}
		}
		if (tooBig)
		{
			return false;
		}
		nonLeafCount = cells.Count;
		treeLike = (logs == nonLeafCount);
		anchored = supportedBelow || (!treeLike && supportedSide);
		if (anchored)
		{
			return true;
		}

		// Leaves belong to the structure when they are within a few cells of one of its logs.
		if (logs > 0)
		{
			Dictionary<Point3, int> distance = new Dictionary<Point3, int>();
			Queue<Point3> leafQueue = new Queue<Point3>();
			for (int i = 0; i < nonLeafCount; i++)
			{
				if (IsLog(terrain.GetCellValue(cells[i].X, cells[i].Y, cells[i].Z)))
				{
					distance[cells[i]] = 0;
					leafQueue.Enqueue(cells[i]);
				}
			}
			while (leafQueue.Count > 0)
			{
				Point3 p = leafQueue.Dequeue();
				int d = distance[p];
				if (d >= SubLevelConfig.MaxLeafDistance)
				{
					continue;
				}
				for (int i = 0; i < 6; i++)
				{
					Point3 n = new Point3(p.X + Neighbors6[i].X, p.Y + Neighbors6[i].Y, p.Z + Neighbors6[i].Z);
					if (n.Y < 0 || n.Y > 255 || distance.ContainsKey(n) || claimed.Contains(n))
					{
						continue;
					}
					if (IsLeaves(terrain.GetCellValue(n.X, n.Y, n.Z)))
					{
						distance[n] = d + 1;
						claimed.Add(n);
						cells.Add(n);
						if (cells.Count > SubLevelConfig.MaxBlocksPerBody)
						{
							return false;
						}
						leafQueue.Enqueue(n);
					}
				}
			}
		}
		return true;
	}
}

// One physics body made of terrain blocks.
public sealed class SubLevel
{
	public RigidBody3D Body;
	public int Count;
	public int[] Values;        // block values
	public PVec3[] Local;       // block centers relative to the center of mass (local space)
	public bool TreeMode;
	public int Humidity;
	public int Temperature;
	public ShipData Ship;       // non-null = a piloted airship
}

public sealed class ShipData
{
	public ComponentPlayer Pilot;
	public PVec3 ForwardLocal;           // ship forward / right in body-local space (initial frame = world axes)
	public PVec3 RightLocal;
	public PVec3 EnvelopeCentroidLocal;
	public PVec3 SeatLocal;
	public float EnvelopeFraction;
	public float EngineFraction;
}

public class SubLevelPhysicsLoader : ModLoader, IPhysWorld
{
	public static SubLevelPhysicsLoader Instance;

	SubsystemTerrain m_subsystemTerrain;
	Terrain m_terrain;
	SubsystemPickables m_subsystemPickables;
	SubsystemBlockBehaviors m_subsystemBlockBehaviors;
	SubsystemTime m_subsystemTime;

	readonly List<SubLevel> m_levels = new List<SubLevel>();
	readonly List<BlockDropValue> m_dropValues = new List<BlockDropValue>();
	PrimitivesRenderer3D m_renderer;
	DrawBlockEnvironmentData m_env;
	bool m_drawErrorLogged;

	public override void __ModInitialize()
	{
		ModsManager.RegisterHook("OnProjectLoaded", this);
		ModsManager.RegisterHook("OnProjectDisposed", this);
		ModsManager.RegisterHook("OnBlockDug", this);
		ModsManager.RegisterHook("OnMinerPlace", this);
		ModsManager.RegisterHook("SubsystemUpdate", this);
		Instance = this;
		InstallPatches();
	}

	// The API package has no ModLoader hooks for terrain drawing and saving, so patch the two game methods with Harmony
	// (looked up by name at runtime, so this compiles against any API version).
	static void InstallPatches()
	{
		try
		{
			Harmony harmony = new Harmony("com.mrzgaming.sublevelphysics");
			MethodInfo draw = AccessTools.Method(typeof(SubsystemTerrain), "Draw", new Type[] { typeof(Camera), typeof(int) });
			if (draw != null)
			{
				harmony.Patch(draw, null, new HarmonyMethod(typeof(SubLevelPatches).GetMethod("DrawPostfix")));
			}
			else
			{
				Log.Error("SubLevel: SubsystemTerrain.Draw not found, bodies will not be drawn");
			}
			MethodInfo save = AccessTools.Method(typeof(SubsystemTerrain), "Save", new Type[] { typeof(ValuesDictionary) });
			if (save != null)
			{
				harmony.Patch(save, new HarmonyMethod(typeof(SubLevelPatches).GetMethod("SavePrefix")), null);
			}
			else
			{
				Log.Error("SubLevel: SubsystemTerrain.Save not found, bodies will not be settled on save");
			}
		}
		catch (Exception ex)
		{
			Log.Error("SubLevel: Harmony patching failed: " + ex);
		}
	}

	public override void OnProjectLoaded(Project project)
	{
		m_levels.Clear();
		m_subsystemTerrain = project.FindSubsystem<SubsystemTerrain>(true);
		m_terrain = m_subsystemTerrain.Terrain;
		m_subsystemPickables = project.FindSubsystem<SubsystemPickables>(true);
		m_subsystemBlockBehaviors = project.FindSubsystem<SubsystemBlockBehaviors>(true);
		m_subsystemTime = project.FindSubsystem<SubsystemTime>(true);
	}

	public override void OnProjectDisposed()
	{
		m_levels.Clear();
		m_subsystemTerrain = null;
		m_terrain = null;
		m_subsystemPickables = null;
		m_subsystemBlockBehaviors = null;
		m_subsystemTime = null;
	}

	// IPhysWorld
	public bool IsSolid(int x, int y, int z)
	{
		if (y < 0)
		{
			return true;
		}
		if (y > 255)
		{
			return false;
		}
		return StructureScanner.IsSolid(m_terrain.GetCellValue(x, y, z));
	}

	// ---------------------------------------------------------------- trigger

	public override void OnBlockDug(ComponentMiner componentMiner, BlockPlacementData digValue, int cellValue, ref int durabilityReduction, ref bool mute, ref int playerDataDugAdd)
	{
		if (m_subsystemTerrain == null)
		{
			return;
		}
		try
		{
			Point3 dug = new Point3(digValue.CellFace.X, digValue.CellFace.Y, digValue.CellFace.Z);
			CheckCollapse(componentMiner, dug);
		}
		catch (Exception ex)
		{
			Log.Error("SubLevel: collapse check failed: " + ex);
		}
	}

	static readonly Point3[] SeedOffsets =
	{
		new Point3(0, 1, 0), new Point3(1, 0, 0), new Point3(-1, 0, 0),
		new Point3(0, 0, 1), new Point3(0, 0, -1), new Point3(0, -1, 0)
	};

	void CheckCollapse(ComponentMiner miner, Point3 dug)
	{
		HashSet<Point3> claimed = new HashSet<Point3>();
		for (int s = 0; s < SeedOffsets.Length; s++)
		{
			if (m_levels.Count >= SubLevelConfig.MaxBodies)
			{
				return;
			}
			Point3 seed = new Point3(dug.X + SeedOffsets[s].X, dug.Y + SeedOffsets[s].Y, dug.Z + SeedOffsets[s].Z);
			if (claimed.Contains(seed))
			{
				continue;
			}
			int v = m_terrain.GetCellValue(seed.X, seed.Y, seed.Z);
			if (!StructureScanner.IsStructural(v) || StructureScanner.IsLeaves(v))
			{
				continue;
			}
			List<Point3> cells;
			int nonLeaf;
			bool treeLike;
			bool anchored;
			if (!StructureScanner.Scan(m_terrain, seed, claimed, out cells, out nonLeaf, out treeLike, out anchored) || anchored)
			{
				continue;
			}
			StartBody(cells, nonLeaf, treeLike, dug, miner);
		}
	}

	SubLevel StartBody(List<Point3> cells, int nonLeaf, bool treeLike, Point3 dug, ComponentMiner miner)
	{
		int count = cells.Count;
		int[] cx = new int[count];
		int[] cy = new int[count];
		int[] cz = new int[count];
		float[] mass = new float[count];
		int[] values = new int[count];
		for (int i = 0; i < count; i++)
		{
			Point3 p = cells[i];
			int v = m_terrain.GetCellValue(p.X, p.Y, p.Z);
			cx[i] = p.X;
			cy[i] = p.Y;
			cz[i] = p.Z;
			values[i] = v;
			mass[i] = Math.Max(0.1f, BlocksManager.Blocks[Terrain.ExtractContents(v)].GetDensity(v));
		}

		RigidBody3D body = new RigidBody3D(cx, cy, cz, mass, count);

		SubLevel level = new SubLevel();
		level.Body = body;
		level.Count = count;
		level.Values = values;
		level.TreeMode = treeLike;
		level.Local = new PVec3[count];
		for (int i = 0; i < count; i++)
		{
			level.Local[i] = new PVec3(cx[i] + 0.5f - body.CenterOfMass.X, cy[i] + 0.5f - body.CenterOfMass.Y, cz[i] + 0.5f - body.CenterOfMass.Z);
		}
		level.Humidity = m_terrain.GetSeasonalHumidity(dug.X, dug.Z);
		level.Temperature = m_terrain.GetSeasonalTemperature(dug.X, dug.Z);

		if (treeLike)
		{
			SetupTree(level, cells, nonLeaf, dug, miner);
		}

		// Lift the blocks out of the terrain.
		for (int i = 0; i < count; i++)
		{
			m_subsystemTerrain.ChangeCell(cx[i], cy[i], cz[i], 0, true);
		}
		m_levels.Add(level);
		return level;
	}

	// A felled tree pivots about the back edge of its lowest log (pin joint) and is pushed away from the player.
	void SetupTree(SubLevel level, List<Point3> cells, int nonLeaf, Point3 dug, ComponentMiner miner)
	{
		// lowest log, the one closest to the cut if several
		Point3 baseCell = cells[0];
		long bestScore = long.MaxValue;
		for (int i = 0; i < nonLeaf; i++)
		{
			long dx = cells[i].X - dug.X;
			long dz = cells[i].Z - dug.Z;
			long score = (long)cells[i].Y * 100000L + dx * dx + dz * dz;
			if (score < bestScore)
			{
				bestScore = score;
				baseCell = cells[i];
			}
		}

		// direction: horizontally away from the player, along one axis
		Vector3 pp = miner.ComponentCreature.ComponentBody.Position;
		float ddx = baseCell.X + 0.5f - pp.X;
		float ddz = baseCell.Z + 0.5f - pp.Z;
		float dirX = 0f;
		float dirZ = 0f;
		if (MathF.Abs(ddx) >= MathF.Abs(ddz))
		{
			dirX = (ddx >= 0f) ? 1f : -1f;
		}
		else
		{
			dirZ = (ddz >= 0f) ? 1f : -1f;
		}

		// hinge height: first solid support below the base (stump top or ground)
		int hy = baseCell.Y;
		bool found = false;
		for (int k = 0; k < 5; k++)
		{
			if (IsSolid(baseCell.X, hy - 1, baseCell.Z))
			{
				found = true;
				break;
			}
			hy--;
		}
		RigidBody3D body = level.Body;
		// angular velocity axis = up x dir: the top of the tree moves along dir
		PVec3 axis = new PVec3(dirZ, 0f, -dirX);
		body.AngularVelocity = axis * SubLevelConfig.TreeKick;
		if (found)
		{
			PVec3 hinge = new PVec3(baseCell.X + 0.5f - dirX * 0.5f, hy, baseCell.Z + 0.5f - dirZ * 0.5f);
			body.PinEnabled = true;
			body.PinWorld = hinge;
			body.PinLocal = hinge - body.CenterOfMass;
			body.Velocity = PVec3.Cross(body.AngularVelocity, body.CenterOfMass - hinge);
		}
	}

	// ---------------------------------------------------------------- airship (Create Aeronautics style)

	static bool IsHelm(int value) { return BlocksManager.Blocks[Terrain.ExtractContents(value)] is CraftingTableBlock; }

	static bool IsEnvelope(int value) { return BlocksManager.Blocks[Terrain.ExtractContents(value)] is CanvasBlock; }

	static bool IsEngine(int value)
	{
		Block b = BlocksManager.Blocks[Terrain.ExtractContents(value)];
		return b is FurnaceBlock || b is LitFurnaceBlock;
	}

	static void Say(ComponentPlayer player, string text)
	{
		if (player != null && player.ComponentGui != null)
		{
			player.ComponentGui.DisplaySmallMessage(text, Color.White, true, false);
		}
	}

	// Placing a Crafting Table (the helm) next to a structure that has balloon (canvas) and engine (furnace) blocks
	// assembles the whole connected structure into an airship. Without them the table is placed normally.
	public override void OnMinerPlace(ComponentMiner miner, TerrainRaycastResult raycastResult, int x, int y, int z, int value, out bool placed)
	{
		placed = false;
		if (m_subsystemTerrain == null || !IsHelm(value))
		{
			return;
		}
		try
		{
			placed = TryAssembleShip(miner, x, y, z, value);
		}
		catch (Exception ex)
		{
			Log.Error("Ship: assemble failed: " + ex);
		}
	}

	bool TryAssembleShip(ComponentMiner miner, int x, int y, int z, int helmValue)
	{
		ComponentPlayer pilot = miner.ComponentPlayer;
		if (pilot == null || m_levels.Count >= SubLevelConfig.MaxBodies)
		{
			return false;
		}
		Point3 helm = new Point3(x, y, z);
		HashSet<Point3> claimed = new HashSet<Point3>();
		claimed.Add(helm);
		List<Point3> cells = new List<Point3>();
		for (int s = 0; s < SeedOffsets.Length; s++)
		{
			Point3 seed = new Point3(x + SeedOffsets[s].X, y + SeedOffsets[s].Y, z + SeedOffsets[s].Z);
			if (claimed.Contains(seed))
			{
				continue;
			}
			int v = m_terrain.GetCellValue(seed.X, seed.Y, seed.Z);
			if (!StructureScanner.IsStructural(v) || StructureScanner.IsLeaves(v))
			{
				continue;
			}
			List<Point3> part;
			int nonLeaf;
			bool treeLike;
			bool anchored;
			if (!StructureScanner.Scan(m_terrain, seed, claimed, out part, out nonLeaf, out treeLike, out anchored))
			{
				return false;
			}
			for (int i = 0; i < nonLeaf; i++)
			{
				cells.Add(part[i]);
			}
		}
		if (cells.Count == 0 || cells.Count + 1 > SubLevelConfig.MaxBlocksPerBody)
		{
			return false;
		}
		int envelope = 0;
		int engines = 0;
		for (int i = 0; i < cells.Count; i++)
		{
			int v = m_terrain.GetCellValue(cells[i].X, cells[i].Y, cells[i].Z);
			if (IsEnvelope(v))
			{
				envelope++;
			}
			else if (IsEngine(v))
			{
				engines++;
			}
		}
		if (envelope == 0)
		{
			return false; // an ordinary crafting table
		}
		if (engines == 0)
		{
			Say(pilot, "Kapal butuh mesin (tungku).");
			return false;
		}

		// the helm becomes part of the structure
		m_subsystemTerrain.ChangeCell(x, y, z, helmValue, true);
		cells.Add(helm);
		SubLevel level = StartBody(cells, cells.Count, false, helm, miner);
		if (level == null)
		{
			return false;
		}

		ShipData ship = new ShipData();
		ship.Pilot = pilot;
		Matrix eye = Matrix.CreateFromQuaternion(miner.ComponentCreature.ComponentCreatureModel.EyeRotation);
		Vector3 fwd = eye.Forward;
		Vector3 rgt = eye.Right;
		if (MathF.Abs(fwd.X) >= MathF.Abs(fwd.Z))
		{
			ship.ForwardLocal = new PVec3(fwd.X >= 0f ? 1f : -1f, 0f, 0f);
			ship.RightLocal = new PVec3(0f, 0f, rgt.Z >= 0f ? 1f : -1f);
		}
		else
		{
			ship.ForwardLocal = new PVec3(0f, 0f, fwd.Z >= 0f ? 1f : -1f);
			ship.RightLocal = new PVec3(rgt.X >= 0f ? 1f : -1f, 0f, 0f);
		}

		PVec3 envSum = default(PVec3);
		int envCount = 0;
		int engineCount = 0;
		for (int i = 0; i < level.Count; i++)
		{
			int v = level.Values[i];
			if (IsEnvelope(v))
			{
				envSum = envSum + level.Local[i];
				envCount++;
			}
			else if (IsEngine(v))
			{
				engineCount++;
			}
			else if (IsHelm(v))
			{
				ship.SeatLocal = level.Local[i] + new PVec3(0f, 0.5f, 0f);
			}
		}
		ship.EnvelopeCentroidLocal = envSum * (1f / Math.Max(1, envCount));
		ship.EnvelopeFraction = (float)envCount / level.Count;
		ship.EngineFraction = (float)engineCount / level.Count;

		level.Ship = ship;
		level.Body.NoSleep = true;
		level.Body.LinearDamping = ShipConfig.LinearDamping;
		level.Body.AngularDamping = ShipConfig.AngularDamping;

		int liftPercent = (int)(100f * ship.EnvelopeFraction / ShipConfig.RequiredEnvelopeFraction);
		Say(pilot, "Kapal dirakit: " + level.Count + " blok, balon " + envCount + ", mesin " + engineCount + ", daya angkat " + liftPercent + "%. Joystick: maju/belok, Lompat: naik, Jongkok: parkir.");
		return true;
	}

	bool ShipDismountRequested(SubLevel level)
	{
		ComponentPlayer pilot = level.Ship.Pilot;
		if (pilot == null || pilot.ComponentBody == null || pilot.ComponentHealth.Health <= 0f)
		{
			return true;
		}
		if (level.Body.Age < ShipConfig.DismountGrace)
		{
			return false;
		}
		PlayerInput input = pilot.ComponentInput.PlayerInput;
		return input.ToggleCrouch || input.ToggleMount;
	}

	void ApplyShipControls(SubLevel level, float dt)
	{
		ShipData ship = level.Ship;
		RigidBody3D body = level.Body;
		PlayerInput input = ship.Pilot.ComponentInput.PlayerInput;
		float forward = Math.Max(-1f, Math.Min(1f, input.Move.Z));
		float turn = Math.Max(-1f, Math.Min(1f, input.Move.X));

		PMat3 R = body.Rotation.ToMat3();
		PVec3 f = R.Mul(ship.ForwardLocal);
		PVec3 r = R.Mul(ship.RightLocal);
		PVec3 worldUp = new PVec3(0f, 1f, 0f);

		// lift from the balloon blocks, applied at their centroid (so an uneven balloon tilts the ship)
		float weight = body.Mass * body.Gravity;
		float capacity = weight * (ship.EnvelopeFraction / ShipConfig.RequiredEnvelopeFraction);
		float factor = ShipConfig.HoverFactor + (input.Jump ? ShipConfig.AscendBonus : 0f);
		float lift = Math.Min(capacity, weight * factor);
		body.AddForceAtPoint(new PVec3(0f, lift, 0f), body.LocalToWorld(ship.EnvelopeCentroidLocal));
		body.AddForce(new PVec3(0f, -body.Velocity.Y * body.Mass * ShipConfig.VerticalDrag, 0f));

		// thrust from the engines along the ship's forward axis
		float accel = Math.Min(ShipConfig.MaxThrustAccel, ship.EngineFraction / ShipConfig.RequiredEngineFraction * ShipConfig.ThrustAccelAtRequired);
		body.AddForce(f * (forward * accel * body.Mass));

		// steering: turn the ship about the vertical axis toward its right side
		PVec3 axis = PVec3.Cross(f, r);
		float sign = (axis.Y >= 0f) ? 1f : -1f;
		float targetYaw = sign * turn * ShipConfig.YawRate;
		PVec3 w = body.AngularVelocity;
		w.Y += (targetYaw - w.Y) * Math.Min(1f, ShipConfig.YawResponse * dt);
		body.AngularVelocity = w;

		// keep the ship level (pitch and roll)
		PVec3 upW = R.Mul(worldUp);
		PVec3 righting = PVec3.Cross(upW, worldUp);
		PVec3 wHorizontal = body.AngularVelocity - worldUp * body.AngularVelocity.Y;
		body.AddTorque((righting * ShipConfig.RightingAccel - wHorizontal * ShipConfig.TiltDamping) * body.InertiaScale);
	}

	// The pilot rides at the helm.
	void AttachPilot(SubLevel level)
	{
		ComponentBody pilotBody = level.Ship.Pilot.ComponentBody;
		PVec3 seat = level.Body.LocalToWorld(level.Ship.SeatLocal);
		pilotBody.Position = new Vector3(seat.X, seat.Y, seat.Z);
		pilotBody.Velocity = Vector3.Zero;
	}

	// Parking: the ship is put back into the terrain as blocks right where it is.
	void Dismount(SubLevel level)
	{
		ComponentPlayer pilot = level.Ship.Pilot;
		PVec3 seat = level.Body.LocalToWorld(level.Ship.SeatLocal);
		level.Ship = null;
		Settle(level);
		if (pilot != null && pilot.ComponentBody != null)
		{
			pilot.ComponentBody.Position = new Vector3(seat.X, seat.Y + 0.6f, seat.Z);
			pilot.ComponentBody.Velocity = Vector3.Zero;
			Say(pilot, "Kapal diparkir.");
		}
	}

	// ---------------------------------------------------------------- simulation

	public override void SubsystemUpdate(float dt)
	{
		if (m_levels.Count == 0 || m_subsystemTime == null)
		{
			return;
		}
		float step = Math.Min(m_subsystemTime.GameTimeDelta, 0.05f);
		if (step <= 0f)
		{
			return;
		}
		for (int i = m_levels.Count - 1; i >= 0; i--)
		{
			SubLevel level = m_levels[i];
			try
			{
				RigidBody3D body = level.Body;
				if (m_terrain.GetChunkAtCell((int)MathF.Floor(body.Position.X), (int)MathF.Floor(body.Position.Z)) == null)
				{
					continue; // chunk not loaded: freeze
				}
				if (level.Ship != null && ShipDismountRequested(level))
				{
					m_levels.RemoveAt(i);
					Dismount(level);
					continue;
				}
				if (level.Ship != null)
				{
					ApplyShipControls(level, step);
				}
				body.Step(step, this);
				if (level.Ship != null)
				{
					AttachPilot(level);
				}
				if (body.PinEnabled && body.Rotation.ToMat3().M11 < MathF.Cos(SubLevelConfig.TreeReleaseTilt))
				{
					body.PinEnabled = false;
				}
				if (body.Position.Y < -30f)
				{
					m_levels.RemoveAt(i);
					continue;
				}
				if (level.Ship == null && (body.Sleeping || body.Age > SubLevelConfig.MaxBodyAge))
				{
					m_levels.RemoveAt(i);
					Settle(level);
				}
			}
			catch (Exception ex)
			{
				Log.Error("SubLevel: update failed: " + ex);
				m_levels.RemoveAt(i);
			}
		}
	}

	// Saving the world while bodies are in flight would lose their blocks: settle them first.
	public void BeforeTerrainSaveHook()
	{
		for (int i = m_levels.Count - 1; i >= 0; i--)
		{
			SubLevel level = m_levels[i];
			m_levels.RemoveAt(i);
			try
			{
				Settle(level);
			}
			catch (Exception ex)
			{
				Log.Error("SubLevel: settle on save failed: " + ex);
			}
		}
	}

	// ---------------------------------------------------------------- settle (put back as blocks or drop)

	// Rounds a rotation to the nearest of the 24 axis-aligned rotations. S[row*3+col] in {-1,0,1}.
	static bool SnapRotation(PMat3 R, int[] S)
	{
		float[] m = { R.M00, R.M01, R.M02, R.M10, R.M11, R.M12, R.M20, R.M21, R.M22 };
		bool[] rowUsed = new bool[3];
		for (int i = 0; i < 9; i++)
		{
			S[i] = 0;
		}
		for (int col = 0; col < 3; col++)
		{
			int bestRow = 0;
			float bestAbs = -1f;
			for (int row = 0; row < 3; row++)
			{
				float a = MathF.Abs(m[row * 3 + col]);
				if (a > bestAbs)
				{
					bestAbs = a;
					bestRow = row;
				}
			}
			if (rowUsed[bestRow])
			{
				return false;
			}
			rowUsed[bestRow] = true;
			S[bestRow * 3 + col] = (m[bestRow * 3 + col] >= 0f) ? 1 : -1;
		}
		int det = S[0] * (S[4] * S[8] - S[5] * S[7]) - S[1] * (S[3] * S[8] - S[5] * S[6]) + S[2] * (S[3] * S[7] - S[4] * S[6]);
		return det == 1;
	}

	static int RemapValue(int value, int[] S)
	{
		if (!(BlocksManager.Blocks[Terrain.ExtractContents(value)] is WoodBlock))
		{
			return value;
		}
		int data = Terrain.ExtractData(value);
		int cutFace = WoodBlock.GetCutFace(data);
		int localAxis = (cutFace == 4 || cutFace == 5) ? 1 : ((cutFace == 0 || cutFace == 2) ? 2 : 0);
		int newAxis = localAxis;
		for (int row = 0; row < 3; row++)
		{
			if (S[row * 3 + localAxis] != 0)
			{
				newAxis = row;
			}
		}
		int newCutFace = (newAxis == 1) ? 4 : ((newAxis == 2) ? 0 : 1);
		return Terrain.MakeBlockValue(Terrain.ExtractContents(value), 0, WoodBlock.SetCutFace(data, newCutFace));
	}

	bool IsReplaceable(int x, int y, int z)
	{
		if (y < 0 || y > 255)
		{
			return false;
		}
		return !StructureScanner.IsSolid(m_terrain.GetCellValue(x, y, z));
	}

	void Settle(SubLevel level)
	{
		RigidBody3D body = level.Body;
		int count = level.Count;
		int[] S = new int[9];
		if (!SubLevelConfig.RestoreBlocks || !SnapRotation(body.Rotation.ToMat3(), S))
		{
			DropAll(level);
			return;
		}

		// rotated block offsets and a position shift so the blocks land exactly on the cell lattice
		PVec3[] sw = new PVec3[count];
		for (int i = 0; i < count; i++)
		{
			PVec3 l = level.Local[i];
			sw[i] = new PVec3(
				S[0] * l.X + S[1] * l.Y + S[2] * l.Z,
				S[3] * l.X + S[4] * l.Y + S[5] * l.Z,
				S[6] * l.X + S[7] * l.Y + S[8] * l.Z);
		}
		PVec3 p0 = body.Position + sw[0];
		PVec3 origin = new PVec3(
			body.Position.X + (MathF.Floor(p0.X) + 0.5f - p0.X),
			body.Position.Y + (MathF.Floor(p0.Y) + 0.5f - p0.Y),
			body.Position.Z + (MathF.Floor(p0.Z) + 0.5f - p0.Z));

		// try to place without conflicts, moving up if the body is slightly sunk into the terrain
		int bestDy = 0;
		int bestConflicts = int.MaxValue;
		for (int dy = 0; dy <= 2 && bestConflicts > 0; dy++)
		{
			int conflicts = 0;
			for (int i = 0; i < count; i++)
			{
				int x = (int)MathF.Round(origin.X + sw[i].X - 0.5f);
				int y = (int)MathF.Round(origin.Y + sw[i].Y - 0.5f) + dy;
				int z = (int)MathF.Round(origin.Z + sw[i].Z - 0.5f);
				if (!IsReplaceable(x, y, z))
				{
					conflicts++;
				}
			}
			if (conflicts < bestConflicts)
			{
				bestConflicts = conflicts;
				bestDy = dy;
			}
		}

		for (int i = 0; i < count; i++)
		{
			int x = (int)MathF.Round(origin.X + sw[i].X - 0.5f);
			int y = (int)MathF.Round(origin.Y + sw[i].Y - 0.5f) + bestDy;
			int z = (int)MathF.Round(origin.Z + sw[i].Z - 0.5f);
			if (IsReplaceable(x, y, z))
			{
				m_subsystemTerrain.ChangeCell(x, y, z, RemapValue(level.Values[i], S), true);
			}
			else
			{
				DropBlock(level.Values[i], new Vector3(x + 0.5f, y + 0.5f, z + 0.5f));
			}
		}
	}

	void DropAll(SubLevel level)
	{
		RigidBody3D body = level.Body;
		PMat3 R = body.Rotation.ToMat3();
		for (int i = 0; i < level.Count; i++)
		{
			PVec3 w = body.Position + R.Mul(level.Local[i]);
			DropBlock(level.Values[i], new Vector3(w.X, w.Y + 0.25f, w.Z));
		}
	}

	// Same drop rules as SubsystemTerrain.DestroyCell.
	void DropBlock(int value, Vector3 position)
	{
		Block block = BlocksManager.Blocks[Terrain.ExtractContents(value)];
		m_dropValues.Clear();
		bool showDebris;
		block.GetDropValues(m_subsystemTerrain, value, 0, 0, m_dropValues, out showDebris);
		int newValue = 0;
		int cx = (int)MathF.Floor(position.X);
		int cy = (int)MathF.Floor(position.Y);
		int cz = (int)MathF.Floor(position.Z);
		for (int i = 0; i < m_dropValues.Count; i++)
		{
			BlockDropValue drop = m_dropValues[i];
			if (drop.Count <= 0)
			{
				continue;
			}
			SubsystemBlockBehavior[] behaviors = m_subsystemBlockBehaviors.GetBlockBehaviors(Terrain.ExtractContents(drop.Value));
			for (int j = 0; j < behaviors.Length; j++)
			{
				behaviors[j].OnItemHarvested(cx, cy, cz, value, ref drop, ref newValue);
			}
			if (drop.Count > 0 && Terrain.ExtractContents(drop.Value) != 0)
			{
				m_subsystemPickables.AddPickable(drop.Value, drop.Count, position, null, null);
			}
		}
	}

	// ---------------------------------------------------------------- rendering

	public void OnTerrainDrawHook(Camera camera, int drawOrder)
	{
		if (drawOrder != 0 || m_levels.Count == 0)
		{
			return;
		}
		try
		{
			DrawLevels(camera);
		}
		catch (Exception ex)
		{
			if (!m_drawErrorLogged)
			{
				m_drawErrorLogged = true;
				Log.Error("SubLevel: draw failed: " + ex);
			}
		}
	}

	void DrawLevels(Camera camera)
	{
		if (m_renderer == null)
		{
			m_renderer = new PrimitivesRenderer3D();
		}
		if (m_env == null)
		{
			m_env = new DrawBlockEnvironmentData();
		}
		m_env.SubsystemTerrain = m_subsystemTerrain;
		bool any = false;
		for (int li = 0; li < m_levels.Count; li++)
		{
			SubLevel level = m_levels[li];
			PVec3 pos = level.Body.Position;
			if (Vector3.Distance(camera.ViewPosition, new Vector3(pos.X, pos.Y, pos.Z)) > SubLevelConfig.MaxVisibleDistance)
			{
				continue;
			}
			PMat3 R = level.Body.Rotation.ToMat3();
			m_env.Humidity = level.Humidity;
			m_env.Temperature = level.Temperature;
			for (int i = 0; i < level.Count; i++)
			{
				PVec3 w = pos + R.Mul(level.Local[i]);
				Matrix matrix = new Matrix(
					R.M00, R.M10, R.M20, 0f,
					R.M01, R.M11, R.M21, 0f,
					R.M02, R.M12, R.M22, 0f,
					w.X, w.Y, w.Z, 1f);
				int light = m_terrain.GetCellLight((int)MathF.Floor(w.X), (int)MathF.Floor(w.Y), (int)MathF.Floor(w.Z));
				m_env.Light = Math.Max(0, Math.Min(15, light));
				m_env.InWorldMatrix = matrix;
				int value = level.Values[i];
				BlocksManager.Blocks[Terrain.ExtractContents(value)].DrawBlock(m_renderer, value, Color.White, 1f, ref matrix, m_env);
			}
			any = true;
		}
		if (any)
		{
			m_renderer.Flush(camera.ViewProjectionMatrix, true, int.MaxValue);
		}
	}
}

// Harmony patches (installed by SubLevelPhysicsLoader.InstallPatches).
public static class SubLevelPatches
{
	public static void DrawPostfix(SubsystemTerrain __instance, Camera camera, int drawOrder)
	{
		SubLevelPhysicsLoader loader = SubLevelPhysicsLoader.Instance;
		if (loader != null)
		{
			loader.OnTerrainDrawHook(camera, drawOrder);
		}
	}

	public static void SavePrefix()
	{
		SubLevelPhysicsLoader loader = SubLevelPhysicsLoader.Instance;
		if (loader != null)
		{
			loader.BeforeTerrainSaveHook();
		}
	}
}
