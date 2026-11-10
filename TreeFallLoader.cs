// Tree Physics Mod - Tahap 2: pohon roboh MIRING dengan rotasi + fisika.
// Target: Survivalcraft 2.4.0.0, API 1.9.5.1. Semua API di sini dicek terhadap source decompile.
//
// Cara kerja:
//  1. OnBlockDug: batang paling bawah ditebang -> kumpulkan batang+dahan+daun (TreeFinder).
//  2. Perencana memilih arah roboh: menjauh dari pemain; kalau terhalang (lereng/dinding), coba arah lain.
//  3. Pohon dimasukkan ke SubsystemMovingBlocks sebagai MovingBlockSet "TreeFall" yang DIAM (tidak digerakkan vanilla).
//     Hook OnMovingBlockSetUpdate menjalankan fisika (TreePhysics.RigidTreeBody) dan melewati update vanilla.
//  4. Gambar (TANPA Harmony): tiap frame fisika, geometri tegak yang disimpan diputar mengelilingi engsel dan ditulis
//     ke set.Geometry; jalur gambar vanilla (DrawMovingBlockSet) lalu menggambarnya apa adanya.
//     Box set dilebarkan supaya uji frustum vanilla tidak memotong pohon yang sedang miring.
//  5. Saat berhenti: set dilepas ke vanilla (set.Stopped), lalu semua blok dijadikan drop item di posisi miring terakhir.
//  Fallback: kalau arah tak ada yang bebas / perencanaan gagal, pohon jatuh lurus seperti Tahap 1.
//
// Perubahan v2.1 (hasil review terhadap source decompile):
//  - set.Stop() dipindah SETELAH ChangeCell (kalau sebelum, game memanggil OnBlockStopMoving, bukan OnBlockStartMoving).
//  - Set disembunyikan sebelum dihapus: vanilla masih menggambar set 4 frame setelah RemoveMovingBlockSet
//    (m_removing), yang tadinya menampilkan pohon TEGAK sesaat di titik tebang (kedip).
//  - MinLeaves: tumpukan batang buatan pemain tanpa daun tidak ikut roboh.
//  - Harmony DIHAPUS (source game tidak membawanya); render lewat geometri yang diputar sendiri.
using System;
using System.Collections.Generic;
using Engine;
using GameEntitySystem;
using TreePhysics;

namespace Game
{
	public static class TreeFallConfig
	{
		public const string SetId = "TreeFall";

		// Batas aman (performa).
		public const int MaxBlocks = 450;          // batang + daun per pohon
		public const int MaxLogs = 120;            // batang per pohon
		public const int MaxLeafDistance = 3;      // daun lebih jauh dari batang ini diabaikan
		public const int MinLeaves = 3;            // kurang dari ini dianggap bukan pohon (mis. tiang kayu buatan pemain)
		public const int MaxHorizontalRadius = 7;  // batang lebih jauh dari titik tebang diabaikan
		public const int MaxConcurrent = 3;        // pohon roboh bersamaan

		// Fisika / bentuk.
		public const float HingeShift = 0.15f;     // engsel digeser dari tengah batang ke arah roboh (0..0.5 blok)
		public const float LogMass = 1.0f;
		public const float LeafMass = 0.2f;
		public const float GoodAngle = 1.0f;       // rad: arah dengan sapuan bebas >= ini langsung dipilih
		public const float MinAcceptAngle = 0.35f; // rad: di bawah ini pohon tidak berotasi (jatuh lurus)
	}

	// Mencari blok mana yang termasuk pohon di atas batang yang ditebang.
	public static class TreeFinder
	{
		static readonly Point3[] Neighbors6 =
		{
			new Point3(1, 0, 0), new Point3(-1, 0, 0), new Point3(0, 1, 0),
			new Point3(0, -1, 0), new Point3(0, 0, 1), new Point3(0, 0, -1)
		};

		public static bool IsLog(int value)
		{
			return BlocksManager.Blocks[Terrain.ExtractContents(value)] is WoodBlock;
		}

		public static bool IsLeaves(int value)
		{
			return BlocksManager.Blocks[Terrain.ExtractContents(value)] is LeavesBlock;
		}

		// cut = sel batang yang baru ditebang. false jika bukan pangkal pohon atau terlalu besar (perilaku vanilla tetap).
		public static bool TryCollect(Terrain terrain, Point3 cut, out List<Point3> logs, out List<Point3> leaves)
		{
			logs = new List<Point3>();
			leaves = new List<Point3>();

			// Hanya batang paling bawah yang dianggap "menebang": sel di bawahnya bukan batang.
			if (IsLog(terrain.GetCellValue(cut.X, cut.Y - 1, cut.Z)))
			{
				return false;
			}

			HashSet<Point3> seen = new HashSet<Point3>();
			Queue<Point3> queue = new Queue<Point3>();
			for (int dx = -1; dx <= 1; dx++)
			{
				for (int dz = -1; dz <= 1; dz++)
				{
					Point3 p = new Point3(cut.X + dx, cut.Y + 1, cut.Z + dz);
					if (IsLog(terrain.GetCellValue(p.X, p.Y, p.Z)) && seen.Add(p))
					{
						queue.Enqueue(p);
					}
				}
			}
			if (queue.Count == 0)
			{
				return false;
			}

			// Batang: flood fill 26-tetangga, tidak pernah di bawah titik tebang.
			while (queue.Count > 0)
			{
				Point3 p = queue.Dequeue();
				logs.Add(p);
				if (logs.Count > TreeFallConfig.MaxLogs)
				{
					return false;
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
							if (n.Y <= cut.Y || n.Y > 255)
							{
								continue;
							}
							if (Math.Abs(n.X - cut.X) > TreeFallConfig.MaxHorizontalRadius || Math.Abs(n.Z - cut.Z) > TreeFallConfig.MaxHorizontalRadius)
							{
								continue;
							}
							if (seen.Contains(n))
							{
								continue;
							}
							if (IsLog(terrain.GetCellValue(n.X, n.Y, n.Z)))
							{
								seen.Add(n);
								queue.Enqueue(n);
							}
						}
					}
				}
			}

			// Daun: flood fill 6-tetangga dari semua batang, jarak terbatas.
			Dictionary<Point3, int> distance = new Dictionary<Point3, int>();
			Queue<Point3> leafQueue = new Queue<Point3>();
			for (int i = 0; i < logs.Count; i++)
			{
				distance[logs[i]] = 0;
				leafQueue.Enqueue(logs[i]);
			}
			while (leafQueue.Count > 0)
			{
				Point3 p = leafQueue.Dequeue();
				int d = distance[p];
				if (d >= TreeFallConfig.MaxLeafDistance)
				{
					continue;
				}
				for (int i = 0; i < Neighbors6.Length; i++)
				{
					Point3 n = new Point3(p.X + Neighbors6[i].X, p.Y + Neighbors6[i].Y, p.Z + Neighbors6[i].Z);
					if (n.Y <= cut.Y || n.Y > 255 || distance.ContainsKey(n))
					{
						continue;
					}
					if (IsLeaves(terrain.GetCellValue(n.X, n.Y, n.Z)))
					{
						distance[n] = d + 1;
						leaves.Add(n);
						if (logs.Count + leaves.Count > TreeFallConfig.MaxBlocks)
						{
							return false;
						}
						leafQueue.Enqueue(n);
					}
				}
			}
			return leaves.Count >= TreeFallConfig.MinLeaves;
		}
	}

	// Status satu pohon yang sedang roboh miring.
	//
	// Cara menggambar TANPA Harmony: vanilla (SubsystemMovingBlocks.DrawMovingBlockSet) menggambar set.Geometry yang
	// digeser sebesar set.Position + set.GeometryOffset. Kita menyimpan salinan geometri tegak (m_base), lalu tiap
	// frame fisika kita mengisi ulang set.Geometry dengan vertex yang sudah diputar mengelilingi engsel. Hasilnya
	// digambar oleh jalur vanilla biasa (shader, kabut, urutan gambar semuanya milik game).
	public sealed class FallingTree
	{
		sealed class BaseSubset
		{
			public TerrainVertex[] Vertices;
			public int[] Indices;
		}

		sealed class BaseTexture
		{
			public Engine.Graphics.Texture2D Texture;
			public BaseSubset[] Subsets;
		}

		public RigidTreeBody Body;
		public float DirX;
		public float DirZ;
		public Vector3 HingeOffset;   // relatif terhadap set.Position
		public int[] Values;          // nilai blok, urutan sama dengan Body
		public Point3[] Offsets;      // offset blok relatif titik tebang
		public int ToolLevel;
		public bool Finished;         // fisika selesai; set dilepas ke vanilla untuk dihentikan
		public bool DrawFailed;

		BaseTexture[] m_base;
		float m_lastAngle = -100f;

		// Dipanggil sekali setelah blok dihapus dari terrain. Menyimpan geometri tegak dan melebarkan Box set
		// (hanya dipakai vanilla untuk uji frustum) supaya pohon yang miring tidak terpotong dari layar.
		public void Activate(SubsystemMovingBlocks subsystem, SubsystemMovingBlocks.MovingBlockSet set)
		{
			// Pastikan geometri sudah dibuat dengan Box ASLI dan posisi cache terisi; kalau tidak, vanilla bisa membuatnya
			// ulang nanti dengan Box yang sudah dilebarkan (melebihi kanvas 32x32 milik generator geometri).
			if (set.GeometryGenerationPosition == new Point3(int.MaxValue))
			{
				subsystem.GenerateGeometry(set);
			}
			CaptureBase(set);

			int r = (int)MathF.Ceiling(Body.MaxRadius) + 1;
			int cx = (int)MathF.Floor(HingeOffset.X);
			int cy = (int)MathF.Floor(HingeOffset.Y);
			int cz = (int)MathF.Floor(HingeOffset.Z);
			set.Box = new Box(cx - r, cy - r, cz - r, 2 * r + 1, 2 * r + 1, 2 * r + 1);
			UpdateGeometry(set);
		}

		void CaptureBase(SubsystemMovingBlocks.MovingBlockSet set)
		{
			List<BaseTexture> list = new List<BaseTexture>();
			foreach (KeyValuePair<Engine.Graphics.Texture2D, TerrainGeometry> pair in set.Geometry.Draws)
			{
				TerrainGeometry g = pair.Value;
				BaseTexture bt = new BaseTexture();
				bt.Texture = pair.Key;
				bt.Subsets = new BaseSubset[g.Subsets.Length];
				for (int j = 0; j < g.Subsets.Length; j++)
				{
					TerrainGeometrySubset s = g.Subsets[j];
					BaseSubset bs = new BaseSubset();
					int vc = s.Vertices.Count;
					int ic = s.Indices.Count;
					bs.Vertices = new TerrainVertex[vc];
					bs.Indices = new int[ic];
					if (vc > 0)
					{
						System.Array.Copy(s.Vertices.Array, bs.Vertices, vc);
					}
					if (ic > 0)
					{
						System.Array.Copy(s.Indices.Array, bs.Indices, ic);
					}
					bt.Subsets[j] = bs;
				}
				list.Add(bt);
			}
			m_base = list.ToArray();
		}

		// Isi ulang set.Geometry dengan vertex yang diputar sebesar Body.Angle. Dipanggil tiap Update fisika.
		public void UpdateGeometry(SubsystemMovingBlocks.MovingBlockSet set)
		{
			if (m_base == null || DrawFailed)
			{
				return;
			}
			float angle = Body.Angle;
			if (MathF.Abs(angle - m_lastAngle) < 0.0005f)
			{
				return;
			}
			m_lastAngle = angle;

			Vector3 hinge = set.Position + HingeOffset;
			Vector3 shift = set.Position + set.GeometryOffset;   // vanilla menambahkan ini kembali saat menggambar
			float ox = shift.X - hinge.X;
			float oy = shift.Y - hinge.Y;
			float oz = shift.Z - hinge.Z;
			float c = MathF.Cos(angle);
			float s = MathF.Sin(angle);
			float dx = DirX;
			float dz = DirZ;

			set.Geometry.ClearGeometry();
			for (int t = 0; t < m_base.Length; t++)
			{
				TerrainGeometry g = set.Geometry.GetGeometry(m_base[t].Texture);
				for (int j = 0; j < m_base[t].Subsets.Length; j++)
				{
					BaseSubset bs = m_base[t].Subsets[j];
					if (bs.Vertices.Length == 0 || bs.Indices.Length == 0)
					{
						continue;
					}
					TerrainGeometrySubset dst = g.Subsets[j];
					dst.Indices.Count = bs.Indices.Length;
					System.Array.Copy(bs.Indices, dst.Indices.Array, bs.Indices.Length);
					dst.Vertices.Count = bs.Vertices.Length;
					TerrainVertex[] arr = dst.Vertices.Array;
					for (int i = 0; i < bs.Vertices.Length; i++)
					{
						TerrainVertex v = bs.Vertices[i];
						float rx = v.X + ox;                 // relatif terhadap engsel
						float ry = v.Y + oy;
						float rz = v.Z + oz;
						float u = rx * dx + rz * dz;         // searah roboh
						float side = rx * dz - rz * dx;      // searah sumbu engsel (tidak berubah)
						float u2 = u * c + ry * s;
						float y2 = ry * c - u * s;
						v.X = dx * u2 + dz * side - ox;      // "- o*" = kembali ke koordinat geometri vanilla
						v.Y = y2 - oy;
						v.Z = dz * u2 - dx * side - oz;
						arr[i] = v;
					}
				}
			}
		}
	}

	public class TreeFallLoader : ModLoader
	{
		public static TreeFallLoader Instance;

		SubsystemTerrain m_subsystemTerrain;
		SubsystemMovingBlocks m_subsystemMovingBlocks;
		SubsystemPickables m_subsystemPickables;
		SubsystemBlockBehaviors m_subsystemBlockBehaviors;

		SolidQuery m_solid;
		readonly System.Random m_random = new System.Random();

		// Data sisi per pohon. Tidak disimpan di MovingBlockSet.Tag karena Tag ikut diserialisasi saat world disimpan.
		readonly Dictionary<IMovingBlockSet, FallingTree> m_trees = new Dictionary<IMovingBlockSet, FallingTree>();
		readonly Dictionary<IMovingBlockSet, int> m_toolLevel = new Dictionary<IMovingBlockSet, int>();
		readonly Dictionary<IMovingBlockSet, Dictionary<Point3, int>> m_index = new Dictionary<IMovingBlockSet, Dictionary<Point3, int>>();
		readonly List<BlockDropValue> m_dropValues = new List<BlockDropValue>();

		public override void __ModInitialize()
		{
			Instance = this;
			m_solid = IsSolidAt;
			ModsManager.RegisterHook("OnProjectLoaded", this);
			ModsManager.RegisterHook("OnProjectDisposed", this);
			ModsManager.RegisterHook("OnBlockDug", this);
			ModsManager.RegisterHook("OnMovingBlockSetUpdate", this);
		}

		public override void OnProjectLoaded(Project project)
		{
			Unsubscribe();
			m_subsystemTerrain = project.FindSubsystem<SubsystemTerrain>(true);
			m_subsystemMovingBlocks = project.FindSubsystem<SubsystemMovingBlocks>(true);
			m_subsystemPickables = project.FindSubsystem<SubsystemPickables>(true);
			m_subsystemBlockBehaviors = project.FindSubsystem<SubsystemBlockBehaviors>(true);
			m_subsystemMovingBlocks.Stopped += OnMovingBlocksStopped;
			m_subsystemMovingBlocks.CollidedWithTerrain += OnMovingBlocksCollided;
		}

		public override void OnProjectDisposed()
		{
			Unsubscribe();
			m_subsystemTerrain = null;
			m_subsystemMovingBlocks = null;
			m_subsystemPickables = null;
			m_subsystemBlockBehaviors = null;
		}

		void Unsubscribe()
		{
			if (m_subsystemMovingBlocks != null)
			{
				m_subsystemMovingBlocks.Stopped -= OnMovingBlocksStopped;
				m_subsystemMovingBlocks.CollidedWithTerrain -= OnMovingBlocksCollided;
			}
			m_trees.Clear();
			m_toolLevel.Clear();
			m_index.Clear();
		}

		public FallingTree GetTree(IMovingBlockSet set)
		{
			FallingTree tree;
			return m_trees.TryGetValue(set, out tree) ? tree : null;
		}

		// ---------------------------------------------------------------- mulai roboh

		// Dipanggil ComponentMiner setelah batang dihapus dari terrain.
		public override void OnBlockDug(ComponentMiner componentMiner, BlockPlacementData digValue, int cellValue, ref int durabilityReduction, ref bool mute, ref int playerDataDugAdd)
		{
			if (m_subsystemMovingBlocks == null || !TreeFinder.IsLog(cellValue))
			{
				return;
			}
			try
			{
				TryStartFall(componentMiner, digValue);
			}
			catch (Exception ex)
			{
				Log.Error("TreeFall: failed to start fall: " + ex);
			}
		}

		void TryStartFall(ComponentMiner miner, BlockPlacementData digValue)
		{
			if (CountActive() >= TreeFallConfig.MaxConcurrent)
			{
				return;
			}
			Point3 cut = new Point3(digValue.CellFace.X, digValue.CellFace.Y, digValue.CellFace.Z);
			List<Point3> logs;
			List<Point3> leaves;
			if (!TreeFinder.TryCollect(m_subsystemTerrain.Terrain, cut, out logs, out leaves))
			{
				return;
			}

			List<MovingBlock> blocks = new List<MovingBlock>(logs.Count + leaves.Count);
			int maxOffsetY = 0;
			AddBlocks(blocks, logs, cut, ref maxOffsetY);
			AddBlocks(blocks, leaves, cut, ref maxOffsetY);

			int toolLevel = 0;
			int activeValue = miner.ActiveBlockValue;
			toolLevel = BlocksManager.Blocks[Terrain.ExtractContents(activeValue)].GetToolLevel(activeValue);

			FallingTree tree = null;
			{
				try
				{
					Vector3 player = miner.ComponentCreature.ComponentBody.Position;
					tree = PlanRotation(cut, blocks, player);
				}
				catch (Exception ex)
				{
					Log.Error("TreeFall: planning failed, using straight fall: " + ex);
					tree = null;
				}
			}

			IMovingBlockSet set;
			if (tree != null)
			{
				// Set DIAM di titik tebang; fisika kita yang memutarnya. TargetPosition == Position supaya vanilla
				// langsung menghentikannya begitu kita melepas set (lihat OnMovingBlockSetUpdate).
				set = m_subsystemMovingBlocks.AddMovingBlockSet(
					new Vector3(cut.X, cut.Y, cut.Z),
					new Vector3(cut.X, cut.Y, cut.Z),
					0f, 0f, 0f, new Vector2(0f),
					blocks, TreeFallConfig.SetId, null, true);
			}
			else
			{
				// Tahap 1: jatuh lurus (pola sama dengan CollapsingBlock vanilla).
				set = m_subsystemMovingBlocks.AddMovingBlockSet(
					new Vector3(cut.X, cut.Y, cut.Z),
					new Vector3(cut.X, -(maxOffsetY + 2), cut.Z),
					0f, 10f, 0.7f, new Vector2(0f),
					blocks, TreeFallConfig.SetId, null, true);
			}
			if (set == null)
			{
				return; // ditolak (mis. tumpang tindih dengan set lain); pohon tetap seperti vanilla
			}

			m_toolLevel[set] = toolLevel;
			if (tree != null)
			{
				tree.ToolLevel = toolLevel;
				m_trees[set] = tree;
			}

			// Hapus blok dari terrain; memberi MovingBlock membuat game memanggil OnBlockStartMoving, bukan OnBlockRemoved.
			// PENTING: set.Stop() belum boleh dipanggil di sini. Kalau set sudah "Stopped", SubsystemTerrain.ChangeCellToBehavior
			// memanggil OnBlockStopMoving (perilaku blok BARU = udara) dan melewati OnBlockStartMoving blok kayu/daun.
			for (int i = 0; i < blocks.Count; i++)
			{
				Point3 c = new Point3(cut.X + blocks[i].Offset.X, cut.Y + blocks[i].Offset.Y, cut.Z + blocks[i].Offset.Z);
				m_subsystemTerrain.ChangeCell(c.X, c.Y, c.Z, 0, true, blocks[i]);
			}

			if (tree != null)
			{
				set.Stop(); // set yang "Stop" tidak ikut tabrakan entitas/raycast -> tidak ada kotak hantu di tempat pohon lama
				SubsystemMovingBlocks.MovingBlockSet concrete = set as SubsystemMovingBlocks.MovingBlockSet;
				try
				{
					if (concrete == null)
					{
						throw new InvalidOperationException("moving block set is not SubsystemMovingBlocks.MovingBlockSet");
					}
					tree.Activate(m_subsystemMovingBlocks, concrete);
				}
				catch (Exception ex)
				{
					// Tanpa geometri miring, pohon tidak boleh terlihat tegak sementara fisikanya jalan: sembunyikan, jatuhkan item saja.
					Log.Error("TreeFall: render setup failed, tree will be invisible while falling: " + ex);
					tree.DrawFailed = true;
					if (concrete != null)
					{
						concrete.Geometry.ClearGeometry();
					}
				}
			}
		}

		void AddBlocks(List<MovingBlock> blocks, List<Point3> cells, Point3 origin, ref int maxOffsetY)
		{
			for (int i = 0; i < cells.Count; i++)
			{
				Point3 c = cells[i];
				MovingBlock mb = new MovingBlock
				{
					Value = m_subsystemTerrain.Terrain.GetCellValue(c.X, c.Y, c.Z),
					Offset = new Point3(c.X - origin.X, c.Y - origin.Y, c.Z - origin.Z)
				};
				if (mb.Offset.Y > maxOffsetY)
				{
					maxOffsetY = mb.Offset.Y;
				}
				blocks.Add(mb);
			}
		}

		int CountActive()
		{
			int count = m_trees.Count;
			foreach (IMovingBlockSet s in m_subsystemMovingBlocks.MovingBlockSetEnumerable)
			{
				if (s.Id == TreeFallConfig.SetId && !s.Stopped && !m_trees.ContainsKey(s))
				{
					count++;
				}
			}
			return count;
		}

		// ---------------------------------------------------------------- perencana arah

		// null = tidak ada arah yang cukup bebas (atau belum bisa) -> jatuh lurus.
		FallingTree PlanRotation(Point3 cut, List<MovingBlock> blocks, Vector3 player)
		{
			int n = blocks.Count;
			float[] wx = new float[n];
			float[] wy = new float[n];
			float[] wz = new float[n];
			float[] mass = new float[n];
			int[] values = new int[n];
			Point3[] offsets = new Point3[n];
			for (int i = 0; i < n; i++)
			{
				values[i] = blocks[i].Value;
				offsets[i] = blocks[i].Offset;
				wx[i] = cut.X + blocks[i].Offset.X + 0.5f;
				wy[i] = cut.Y + blocks[i].Offset.Y + 0.5f;
				wz[i] = cut.Z + blocks[i].Offset.Z + 0.5f;
				mass[i] = TreeFinder.IsLog(values[i]) ? TreeFallConfig.LogMass : TreeFallConfig.LeafMass;
			}

			// Arah dasar: menjauh dari pemain.
			float ax = cut.X + 0.5f - player.X;
			float az = cut.Z + 0.5f - player.Z;
			float len = MathF.Sqrt(ax * ax + az * az);
			if (len < 0.2f)
			{
				ax = 1f; az = 0f; len = 1f;
			}
			ax /= len; az /= len;

			float[] offsetsDeg = { 0f, 35f, -35f, 70f, -70f, 110f, -110f, 180f };
			RigidTreeBody best = null;
			float bestClear = -1f;
			for (int k = 0; k < offsetsDeg.Length; k++)
			{
				float rad = offsetsDeg[k] * (MathF.PI / 180f);
				float cs = MathF.Cos(rad);
				float sn = MathF.Sin(rad);
				float dx = ax * cs - az * sn;
				float dz = ax * sn + az * cs;
				RigidTreeBody body = BuildBody(cut, wx, wy, wz, mass, n, dx, dz);
				float clear = body.ClearAngle(m_solid);
				if (clear > bestClear)
				{
					bestClear = clear;
					best = body;
				}
				if (clear >= TreeFallConfig.GoodAngle)
				{
					break; // urutan kandidat = dari yang paling dekat dengan "menjauh dari pemain"
				}
			}
			if (best == null || bestClear < TreeFallConfig.MinAcceptAngle)
			{
				return null;
			}
			FallingTree tree = new FallingTree();
			tree.Body = best;
			tree.DirX = best.DirX;
			tree.DirZ = best.DirZ;
			tree.HingeOffset = new Vector3(0.5f + TreeFallConfig.HingeShift * best.DirX, 1f, 0.5f + TreeFallConfig.HingeShift * best.DirZ);
			tree.Values = values;
			tree.Offsets = offsets;
			return tree;
		}

		RigidTreeBody BuildBody(Point3 cut, float[] wx, float[] wy, float[] wz, float[] mass, int n, float dx, float dz)
		{
			float hx = cut.X + 0.5f + TreeFallConfig.HingeShift * dx;
			float hy = cut.Y + 1f;
			float hz = cut.Z + 0.5f + TreeFallConfig.HingeShift * dz;
			float[] rx = new float[n];
			float[] ry = new float[n];
			float[] rz = new float[n];
			for (int i = 0; i < n; i++)
			{
				rx[i] = wx[i] - hx;
				ry[i] = wy[i] - hy;
				rz[i] = wz[i] - hz;
			}
			return new RigidTreeBody(rx, ry, rz, mass, n, dx, dz, hx, hy, hz, null);
		}

		// Padat bagi pohon yang roboh: terrain yang bisa ditabrak. Pohon lain (batang/daun) ditembus supaya
		// pohon yang roboh tidak tersangkut di tajuk tetangga.
		bool IsSolidAt(float x, float y, float z)
		{
			int cy = (int)MathF.Floor(y);
			if (cy < 0)
			{
				return true;
			}
			if (cy > 255)
			{
				return false;
			}
			int value = m_subsystemTerrain.Terrain.GetCellValue((int)MathF.Floor(x), cy, (int)MathF.Floor(z));
			int contents = Terrain.ExtractContents(value);
			if (contents == 0)
			{
				return false;
			}
			Block block = BlocksManager.Blocks[contents];
			if (block is WoodBlock || block is LeavesBlock)
			{
				return false;
			}
			return block.IsCollidable_(value);
		}

		// ---------------------------------------------------------------- update fisika

		// Dipanggil SubsystemMovingBlocks.Update untuk tiap set, sebelum update vanilla.
		public override void OnMovingBlockSetUpdate(IMovingBlockSet movingBlockSet, SubsystemMovingBlocks subsystemMovingBlocks, bool skippedByOtherMods, out bool skipVanilla)
		{
			skipVanilla = false;
			if (movingBlockSet.Id != TreeFallConfig.SetId)
			{
				return;
			}
			FallingTree tree;
			if (!m_trees.TryGetValue(movingBlockSet, out tree) || tree.Finished)
			{
				// Tanpa status (jatuh lurus, atau world dimuat ulang saat pohon jatuh) atau sudah selesai: biarkan vanilla.
				return;
			}
			skipVanilla = true;
			try
			{
				tree.Body.Step(subsystemMovingBlocks.m_subsystemTime.GameTimeDelta, m_solid);
				UpdateTreeGeometry(movingBlockSet, tree);
				if (tree.Body.Settled)
				{
					// Lepas ke vanilla: TargetPosition == Position -> set langsung "Stopped" di update ini juga
					// (kita sudah mengembalikan skipVanilla=false untuk frame berikutnya).
					tree.Finished = true;
					skipVanilla = false;
				}
			}
			catch (Exception ex)
			{
				Log.Error("TreeFall: physics step failed, finishing tree: " + ex);
				tree.Finished = true;
				skipVanilla = false;
			}
		}

		// Tumbukan terrain dilaporkan untuk setiap sel non-udara di dalam bounding box set, bukan hanya sel yang disentuh blok
		// sungguhan. Pohon jauh lebih lebar dari kolom pasir, jadi pastikan ada blok yang benar-benar tumpang tindih.
		// Hanya dipakai untuk jatuh lurus (Tahap 1).
		void OnMovingBlocksCollided(IMovingBlockSet set, Point3 p)
		{
			if (set.Id != TreeFallConfig.SetId || m_trees.ContainsKey(set))
			{
				return;
			}
			int value = m_subsystemTerrain.Terrain.GetCellValue(p.X, p.Y, p.Z);
			if (Terrain.ExtractContents(value) == 0 || !OverlapsRealBlock(set, p))
			{
				return;
			}
			Block block = BlocksManager.Blocks[Terrain.ExtractContents(value)];
			if (block.IsCollapseSupportBlock(m_subsystemTerrain, value))
			{
				set.Stop();
			}
			else if (block.IsCollapseDestructibleBlock(value))
			{
				m_subsystemTerrain.DestroyCell(0, p.X, p.Y, p.Z, 0, false, false);
			}
		}

		bool OverlapsRealBlock(IMovingBlockSet set, Point3 p)
		{
			Dictionary<Point3, int> index;
			if (!m_index.TryGetValue(set, out index))
			{
				index = new Dictionary<Point3, int>();
				List<MovingBlock> list = set.Blocks;
				for (int i = 0; i < list.Count; i++)
				{
					index[list[i].Offset] = list[i].Value;
				}
				m_index[set] = index;
			}
			Vector3 pos = set.Position;
			int ox = p.X - (int)MathF.Floor(pos.X + 0.5f);
			int oz = p.Z - (int)MathF.Floor(pos.Z + 0.5f);
			int baseOy = (int)MathF.Floor(p.Y - pos.Y);
			for (int oy = baseOy - 1; oy <= baseOy + 1; oy++)
			{
				if (!index.ContainsKey(new Point3(ox, oy, oz)))
				{
					continue;
				}
				float bottom = pos.Y + oy;
				if (bottom < p.Y + 1 - 0.001f && bottom + 1 > p.Y + 0.001f)
				{
					return true;
				}
			}
			return false;
		}

		// ---------------------------------------------------------------- selesai: jadi drop item

		void OnMovingBlocksStopped(IMovingBlockSet set)
		{
			if (set.Id != TreeFallConfig.SetId)
			{
				return;
			}
			try
			{
				int toolLevel;
				m_toolLevel.TryGetValue(set, out toolLevel);
				FallingTree tree;
				if (m_trees.TryGetValue(set, out tree))
				{
					DropRotatedTree(set, tree, toolLevel);
				}
				else
				{
					DropStraight(set, toolLevel);
				}
			}
			catch (Exception ex)
			{
				Log.Error("TreeFall: failed to drop blocks: " + ex);
			}
			finally
			{
				m_trees.Remove(set);
				m_toolLevel.Remove(set);
				m_index.Remove(set);
				HideSet(set);
				m_subsystemMovingBlocks.RemoveMovingBlockSet(set);
			}
		}

		void UpdateTreeGeometry(IMovingBlockSet set, FallingTree tree)
		{
			SubsystemMovingBlocks.MovingBlockSet concrete = set as SubsystemMovingBlocks.MovingBlockSet;
			if (concrete == null || tree.DrawFailed)
			{
				return;
			}
			try
			{
				tree.UpdateGeometry(concrete);
			}
			catch (Exception ex)
			{
				tree.DrawFailed = true;
				concrete.Geometry.ClearGeometry();
				Log.Error("TreeFall: geometry update failed, hiding falling tree: " + ex);
			}
		}

		// RemoveMovingBlockSet menaruh set di m_removing; vanilla masih menggambarnya 4 frame (untuk menutupi jeda mesh terrain).
		// Blok kita jadi drop item, bukan terrain, jadi gambar itu hanya menampilkan pohon "hantu" di posisi terakhir.
		// Kosongkan set (blok, Box, geometri) dan geser jauh di bawah dunia supaya tidak ada yang digambar.
		void HideSet(IMovingBlockSet set)
		{
			SubsystemMovingBlocks.MovingBlockSet concrete = set as SubsystemMovingBlocks.MovingBlockSet;
			if (concrete == null)
			{
				return;
			}
			concrete.Blocks.Clear();
			concrete.UpdateBox();
			concrete.Geometry.ClearGeometry();
			concrete.Position = new Vector3(concrete.Position.X, -10000f, concrete.Position.Z);
		}

		void DropStraight(IMovingBlockSet set, int toolLevel)
		{
			Vector3 pos = set.Position;
			Point3 origin = Terrain.ToCell(MathF.Round(pos.X), MathF.Round(pos.Y), MathF.Round(pos.Z));
			List<MovingBlock> list = set.Blocks;
			for (int i = 0; i < list.Count; i++)
			{
				Point3 c = new Point3(origin.X + list[i].Offset.X, origin.Y + list[i].Offset.Y, origin.Z + list[i].Offset.Z);
				DropBlock(list[i].Value, c, new Vector3(c.X + 0.5f, c.Y + 0.5f, c.Z + 0.5f), toolLevel);
			}
		}

		void DropRotatedTree(IMovingBlockSet set, FallingTree tree, int toolLevel)
		{
			for (int i = 0; i < tree.Values.Length; i++)
			{
				float x;
				float y;
				float z;
				tree.Body.GetBlockCenter(i, out x, out y, out z);
				// Jangan menjatuhkan item di dalam terrain: geser ke atas sampai bebas.
				for (int k = 0; k < 3 && IsSolidAt(x, y, z); k++)
				{
					y += 1f;
				}
				Point3 cell = Terrain.ToCell(x, y, z);
				DropBlock(tree.Values[i], cell, new Vector3(x, y, z), toolLevel);
			}
		}

		// Aturan drop sama dengan SubsystemTerrain.DestroyCell (GetDropValues + OnItemHarvested).
		void DropBlock(int value, Point3 cell, Vector3 position, int toolLevel)
		{
			Block block = BlocksManager.Blocks[Terrain.ExtractContents(value)];
			m_dropValues.Clear();
			bool showDebris;
			block.GetDropValues(m_subsystemTerrain, value, 0, toolLevel, m_dropValues, out showDebris);
			int newValue = 0;
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
					behaviors[j].OnItemHarvested(cell.X, cell.Y, cell.Z, value, ref drop, ref newValue);
				}
				if (drop.Count > 0 && Terrain.ExtractContents(drop.Value) != 0)
				{
					Vector3 velocity = new Vector3(
						(float)(m_random.NextDouble() - 0.5) * 1.5f,
						(float)m_random.NextDouble() * 1.0f,
						(float)(m_random.NextDouble() - 0.5) * 1.5f);
					m_subsystemPickables.AddPickable(drop.Value, drop.Count, position, velocity, null);
				}
			}
		}
	}
}

