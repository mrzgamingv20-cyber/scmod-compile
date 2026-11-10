// TreeFallPhysics.cs - "liblary fisika" untuk pohon roboh.
// Mandiri: TIDAK memakai tipe Engine/Game sama sekali (hanya System), jadi bisa diuji di luar game.
//
// Model: benda tegar (rigid body) berupa kumpulan massa titik (satu per blok) yang berputar pada
// SATU engsel horizontal (pangkal batang). Arah roboh = vektor horizontal (dirX, dirZ).
//   sudut 0 = tegak, sudut pi/2 = rebah mendatar.
//   gravitasi -> torsi dari pusat massa:  alpha = g * (A*cos(a) + B*sin(a)) / I - drag*omega
//       A = sum(m*u)  (u = jarak horizontal massa dari engsel searah roboh)
//       B = sum(m*h)  (h = tinggi massa dari engsel)
//       I = sum(m*(u^2+h^2))  (momen inersia terhadap sumbu engsel)
//   integrasi semi-implisit Euler dengan substep kecil.
//   tumbukan: titik sampel tiap blok dicek ke terrain lewat delegate SolidQuery; saat kena, sudut
//   dicari dengan bisection (tepat sebelum menembus), lalu memantul kecil atau berhenti.
//   berhenti: kena terrain dengan kecepatan sudut kecil, mencapai sudut maksimum, atau waktu habis.
using System;

namespace TreePhysics
{
	// true jika titik dunia (x,y,z) berada di dalam benda padat (terrain).
	public delegate bool SolidQuery(float x, float y, float z);

	public sealed class TreePhysicsSettings
	{
		public float Gravity = 10f;            // m/s^2 (gravitasi Survivalcraft = 10)
		public float AngularDrag = 0.25f;      // redaman sudut (1/s): hambatan udara + ranting
		public float InitialSpin = 0.30f;      // dorongan awal (rad/s) agar mulai miring
		public float Restitution = 0.25f;      // pantulan saat menumbuk tanah
		public float RestSpeed = 0.9f;         // di bawah kecepatan sudut ini tumbukan = berhenti
		public int MaxBounces = 2;
		public float MaxAngle = 1.5707964f;    // pi/2: rebah mendatar
		public float MaxTime = 12f;            // detik; batas pengaman
		public float MaxSubstep = 1f / 90f;
		public float MaxFrameDt = 0.05f;       // dt per frame dipotong ke nilai ini
		public int BisectIterations = 7;
		public float HingeEpsilonInertia = 0.5f;
	}

	public sealed class RigidTreeBody
	{
		readonly float[] m_x;     // posisi massa relatif terhadap engsel (sumbu dunia), sudut 0
		readonly float[] m_y;
		readonly float[] m_z;
		readonly int m_count;
		readonly float m_dx;
		readonly float m_dz;
		readonly float m_hx;      // posisi engsel di dunia
		readonly float m_hy;
		readonly float m_hz;
		readonly TreePhysicsSettings m_s;
		readonly float m_A;
		readonly float m_B;
		readonly float m_I;
		readonly float m_maxRadius;

		public float Angle;
		public float AngularVelocity;
		public float Time;
		public int Bounces;
		public bool Settled;
		public string SettleReason = "";

		public int Count { get { return m_count; } }
		public float MaxRadius { get { return m_maxRadius; } }
		public float DirX { get { return m_dx; } }
		public float DirZ { get { return m_dz; } }

		// relX/relY/relZ: pusat tiap blok relatif terhadap engsel; mass: massa tiap blok.
		// (dirX,dirZ) harus berupa vektor satuan horizontal.
		public RigidTreeBody(float[] relX, float[] relY, float[] relZ, float[] mass, int count,
			float dirX, float dirZ, float hingeX, float hingeY, float hingeZ, TreePhysicsSettings settings)
		{
			m_s = settings ?? new TreePhysicsSettings();
			m_count = count;
			m_x = relX; m_y = relY; m_z = relZ;
			m_dx = dirX; m_dz = dirZ;
			m_hx = hingeX; m_hy = hingeY; m_hz = hingeZ;
			float a = 0f, b = 0f, inertia = m_s.HingeEpsilonInertia, maxR2 = 0f;
			for (int i = 0; i < count; i++)
			{
				float u = relX[i] * dirX + relZ[i] * dirZ;
				float h = relY[i];
				a += mass[i] * u;
				b += mass[i] * h;
				inertia += mass[i] * (u * u + h * h);
				float r2 = relX[i] * relX[i] + relY[i] * relY[i] + relZ[i] * relZ[i];
				if (r2 > maxR2) maxR2 = r2;
			}
			m_A = a; m_B = b; m_I = inertia;
			m_maxRadius = (float)Math.Sqrt(maxR2) + 1f;

			// Titik keseimbangan labil: di bawah sudut ini pohon cenderung "duduk" kembali. Mulai sedikit di atasnya
			// supaya pohon langsung mulai roboh tanpa lompatan yang terlihat.
			float tip = (m_A < 0f && m_B > 0f) ? (float)Math.Atan2(-m_A, m_B) : 0f;
			Angle = Math.Max(tip + 0.012f, 0.025f);
			AngularVelocity = m_s.InitialSpin;
		}

		// Putar vektor relatif (rx,ry,rz) pada sudut tertentu mengelilingi sumbu engsel.
		public void Rotate(float rx, float ry, float rz, float angle, out float ox, out float oy, out float oz)
		{
			float c = (float)Math.Cos(angle);
			float s = (float)Math.Sin(angle);
			RotateCS(rx, ry, rz, c, s, out ox, out oy, out oz);
		}

		void RotateCS(float rx, float ry, float rz, float c, float s, out float ox, out float oy, out float oz)
		{
			float u = rx * m_dx + rz * m_dz;     // searah roboh
			float w = rx * m_dz - rz * m_dx;     // searah sumbu engsel (tidak berubah)
			float u2 = u * c + ry * s;
			oy = ry * c - u * s;
			ox = m_dx * u2 + m_dz * w;
			oz = m_dz * u2 - m_dx * w;
		}

		// Pusat blok ke-i di dunia pada sudut saat ini.
		public void GetBlockCenter(int i, out float wx, out float wy, out float wz)
		{
			float ox, oy, oz;
			Rotate(m_x[i], m_y[i], m_z[i], Angle, out ox, out oy, out oz);
			wx = m_hx + ox; wy = m_hy + oy; wz = m_hz + oz;
		}

		// Cek tumbukan pada sudut tertentu. Tiap blok punya 3 titik sampel:
		// pusat, titik bawah (0.5 di bawah pusat), dan titik depan (0.5 searah roboh).
		public bool Collides(float angle, SolidQuery solid)
		{
			float c = (float)Math.Cos(angle);
			float s = (float)Math.Sin(angle);
			for (int i = 0; i < m_count; i++)
			{
				float ox, oy, oz;
				RotateCS(m_x[i], m_y[i], m_z[i], c, s, out ox, out oy, out oz);
				float wx = m_hx + ox, wy = m_hy + oy, wz = m_hz + oz;
				if (solid(wx, wy - 0.5f, wz) || solid(wx, wy, wz) || solid(wx + m_dx * 0.5f, wy, wz + m_dz * 0.5f))
				{
					return true;
				}
			}
			return false;
		}

		// Untuk perencana arah: sudut terbesar (<= MaxAngle) yang bebas tumbukan, disapu per langkah kecil.
		public float ClearAngle(SolidQuery solid)
		{
			float step = Math.Min(0.1f, Math.Max(0.03f, 0.9f / m_maxRadius));
			float start = Angle;
			if (Collides(start, solid)) return 0f;
			float last = start;
			for (float a = start + step; ; a += step)
			{
				if (a >= m_s.MaxAngle) a = m_s.MaxAngle;
				if (Collides(a, solid)) return last;
				last = a;
				if (a >= m_s.MaxAngle) return last;
			}
		}

		public void Step(float dt, SolidQuery solid)
		{
			if (Settled) return;
			if (dt > m_s.MaxFrameDt) dt = m_s.MaxFrameDt;
			if (dt <= 0f) return;
			if (Time == 0f && Collides(Angle, solid))
			{
				Settle("blocked");
				return;
			}
			int n = (int)Math.Ceiling(dt / m_s.MaxSubstep);
			if (n < 1) n = 1;
			float h = dt / n;
			for (int i = 0; i < n && !Settled; i++)
			{
				Substep(h, solid);
			}
			Time += dt;
			if (!Settled && Time >= m_s.MaxTime)
			{
				Settle("timeout");
			}
		}

		void Substep(float h, SolidQuery solid)
		{
			float a = Angle;
			float alpha = m_s.Gravity * (m_A * (float)Math.Cos(a) + m_B * (float)Math.Sin(a)) / m_I - m_s.AngularDrag * AngularVelocity;
			float w2 = AngularVelocity + alpha * h;
			float a2 = a + w2 * h;
			if (a2 < 0f)
			{
				// terayun kembali ke tunggul: diam
				a2 = 0f; w2 = 0f;
			}
			bool reachedMax = false;
			if (a2 >= m_s.MaxAngle)
			{
				a2 = m_s.MaxAngle;
				reachedMax = true;
			}
			if (Collides(a2, solid))
			{
				float lo = a, hi = a2;
				for (int k = 0; k < m_s.BisectIterations; k++)
				{
					float mid = 0.5f * (lo + hi);
					if (Collides(mid, solid)) hi = mid; else lo = mid;
				}
				Angle = lo;
				if (w2 > m_s.RestSpeed && Bounces < m_s.MaxBounces)
				{
					AngularVelocity = -m_s.Restitution * w2;
					Bounces++;
				}
				else
				{
					AngularVelocity = 0f;
					Settle("contact");
				}
				return;
			}
			Angle = a2;
			AngularVelocity = w2;
			if (reachedMax)
			{
				AngularVelocity = 0f;
				Settle("flat");
			}
		}

		void Settle(string reason)
		{
			Settled = true;
			SettleReason = reason;
		}

		// Energi total (untuk uji): kinetik + potensial (terhadap engsel), tanpa redaman hasilnya konstan.
		public float TotalEnergy()
		{
			float c = (float)Math.Cos(Angle);
			float s = (float)Math.Sin(Angle);
			float potential = m_s.Gravity * (m_B * c - m_A * s);   // g * sum(m * h')
			return 0.5f * m_I * AngularVelocity * AngularVelocity + potential;
		}
	}
}
