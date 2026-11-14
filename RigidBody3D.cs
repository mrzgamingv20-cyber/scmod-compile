// Sub-level physics mod - standalone rigid body library (no Survivalcraft dependencies, own vector/quaternion math).
//
// A RigidBody3D is a group of voxel blocks that moves as one solid object: gravity, linear and angular velocity,
// inertia tensor from the real block layout, collision with a voxel world (via IPhysWorld), friction,
// small bounce, optional pin joint (used as a hinge for falling trees) and sleeping.
//
// Conventions: right-handed math, world = Position + R * local, R from the quaternion in Rotation.
using System;
using System.Collections.Generic;

namespace Game;

public interface IPhysWorld
{
	bool IsSolid(int x, int y, int z);
}

public struct PVec3
{
	public float X;
	public float Y;
	public float Z;

	public PVec3(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	public static PVec3 operator +(PVec3 a, PVec3 b) { return new PVec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
	public static PVec3 operator -(PVec3 a, PVec3 b) { return new PVec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
	public static PVec3 operator -(PVec3 a) { return new PVec3(-a.X, -a.Y, -a.Z); }
	public static PVec3 operator *(PVec3 a, float s) { return new PVec3(a.X * s, a.Y * s, a.Z * s); }
	public static PVec3 operator *(float s, PVec3 a) { return new PVec3(a.X * s, a.Y * s, a.Z * s); }

	public static float Dot(PVec3 a, PVec3 b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }

	public static PVec3 Cross(PVec3 a, PVec3 b)
	{
		return new PVec3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
	}

	public float Length { get { return MathF.Sqrt(X * X + Y * Y + Z * Z); } }
}

public struct PMat3
{
	public float M00, M01, M02;
	public float M10, M11, M12;
	public float M20, M21, M22;

	public PVec3 Mul(PVec3 v)
	{
		return new PVec3(
			M00 * v.X + M01 * v.Y + M02 * v.Z,
			M10 * v.X + M11 * v.Y + M12 * v.Z,
			M20 * v.X + M21 * v.Y + M22 * v.Z);
	}

	public PMat3 Transpose()
	{
		PMat3 t = default(PMat3);
		t.M00 = M00; t.M01 = M10; t.M02 = M20;
		t.M10 = M01; t.M11 = M11; t.M12 = M21;
		t.M20 = M02; t.M21 = M12; t.M22 = M22;
		return t;
	}

	public static PMat3 Mul(PMat3 a, PMat3 b)
	{
		PMat3 r = default(PMat3);
		r.M00 = a.M00 * b.M00 + a.M01 * b.M10 + a.M02 * b.M20;
		r.M01 = a.M00 * b.M01 + a.M01 * b.M11 + a.M02 * b.M21;
		r.M02 = a.M00 * b.M02 + a.M01 * b.M12 + a.M02 * b.M22;
		r.M10 = a.M10 * b.M00 + a.M11 * b.M10 + a.M12 * b.M20;
		r.M11 = a.M10 * b.M01 + a.M11 * b.M11 + a.M12 * b.M21;
		r.M12 = a.M10 * b.M02 + a.M11 * b.M12 + a.M12 * b.M22;
		r.M20 = a.M20 * b.M00 + a.M21 * b.M10 + a.M22 * b.M20;
		r.M21 = a.M20 * b.M01 + a.M21 * b.M11 + a.M22 * b.M21;
		r.M22 = a.M20 * b.M02 + a.M21 * b.M12 + a.M22 * b.M22;
		return r;
	}

	public PMat3 Inverse()
	{
		float c00 = M11 * M22 - M12 * M21;
		float c01 = M12 * M20 - M10 * M22;
		float c02 = M10 * M21 - M11 * M20;
		float det = M00 * c00 + M01 * c01 + M02 * c02;
		if (MathF.Abs(det) < 1e-9f)
		{
			det = 1e-9f;
		}
		float id = 1f / det;
		PMat3 r = default(PMat3);
		r.M00 = c00 * id;
		r.M01 = (M02 * M21 - M01 * M22) * id;
		r.M02 = (M01 * M12 - M02 * M11) * id;
		r.M10 = c01 * id;
		r.M11 = (M00 * M22 - M02 * M20) * id;
		r.M12 = (M02 * M10 - M00 * M12) * id;
		r.M20 = c02 * id;
		r.M21 = (M01 * M20 - M00 * M21) * id;
		r.M22 = (M00 * M11 - M01 * M10) * id;
		return r;
	}
}

public struct PQuat
{
	public float X;
	public float Y;
	public float Z;
	public float W;

	public static readonly PQuat Identity = new PQuat(0f, 0f, 0f, 1f);

	public PQuat(float x, float y, float z, float w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	public static PQuat Mul(PQuat a, PQuat b)
	{
		return new PQuat(
			a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
			a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
			a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
			a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);
	}

	public PQuat Normalized()
	{
		float l = MathF.Sqrt(X * X + Y * Y + Z * Z + W * W);
		if (l < 1e-9f)
		{
			return Identity;
		}
		float i = 1f / l;
		return new PQuat(X * i, Y * i, Z * i, W * i);
	}

	public PMat3 ToMat3()
	{
		float xx = X * X, yy = Y * Y, zz = Z * Z;
		float xy = X * Y, xz = X * Z, yz = Y * Z;
		float wx = W * X, wy = W * Y, wz = W * Z;
		PMat3 m = default(PMat3);
		m.M00 = 1f - 2f * (yy + zz);
		m.M01 = 2f * (xy - wz);
		m.M02 = 2f * (xz + wy);
		m.M10 = 2f * (xy + wz);
		m.M11 = 1f - 2f * (xx + zz);
		m.M12 = 2f * (yz - wx);
		m.M20 = 2f * (xz - wy);
		m.M21 = 2f * (yz + wx);
		m.M22 = 1f - 2f * (xx + yy);
		return m;
	}
}

public sealed class RigidBody3D
{
	// Tunables
	public float Gravity = 10f;
	public float Friction = 0.6f;
	public float Restitution = 0.15f;
	public float LinearDamping = 0.02f;
	public float AngularDamping = 0.25f;
	public float ContactAngularDamping = 1.2f;
	public float MaxSpeed = 40f;
	public float SleepSpeed = 0.15f;
	public float SleepTime = 0.8f;
	public bool NoSleep;                // vehicles never go to sleep

	// State
	public PVec3 Position;          // center of mass, world space
	public PVec3 Velocity;
	public PVec3 AngularVelocity;
	public PQuat Rotation = PQuat.Identity;
	public bool Sleeping;
	public float Age;
	public int ContactCount;

	// Optional pin joint: the body point PinLocal (local space) is held at PinWorld. Used as a hinge for trees.
	public bool PinEnabled;
	public PVec3 PinLocal;
	public PVec3 PinWorld;

	// Mass properties (set by the constructor)
	public readonly PVec3 CenterOfMass;   // initial world position of the center of mass
	public readonly float Mass;
	public readonly float InertiaScale;   // average moment of inertia (for torque = scale * angular acceleration)

	readonly float m_invMass;
	readonly PMat3 m_invInertiaLocal;
	readonly PVec3[] m_samples;           // surface corners, local space (relative to the center of mass)
	readonly PVec3[] m_cn;                // contact normals (scratch)
	readonly PVec3[] m_cr;                // contact arms (scratch)
	int m_contactCount;
	float m_sleepTimer;
	PVec3 m_force;                        // external force/torque, accumulated per frame
	PVec3 m_torque;
	bool m_hasForce;

	static long Key(int x, int y, int z)
	{
		const int off = 1048576;
		return ((long)(x + off) << 42) | ((long)(y + off) << 21) | (long)(z + off);
	}

	// cx/cy/cz: integer cell coordinates of each block, mass: mass of each block. The body starts at rest, upright
	// (identity rotation) exactly where the cells are in the world.
	public RigidBody3D(int[] cx, int[] cy, int[] cz, float[] mass, int count)
	{
		float totalMass = 0f;
		float sx = 0f, sy = 0f, sz = 0f;
		for (int i = 0; i < count; i++)
		{
			totalMass += mass[i];
			sx += mass[i] * (cx[i] + 0.5f);
			sy += mass[i] * (cy[i] + 0.5f);
			sz += mass[i] * (cz[i] + 0.5f);
		}
		if (totalMass < 0.0001f)
		{
			totalMass = 0.0001f;
		}
		Mass = totalMass;
		m_invMass = 1f / totalMass;
		CenterOfMass = new PVec3(sx / totalMass, sy / totalMass, sz / totalMass);
		Position = CenterOfMass;

		// Inertia tensor about the center of mass: sum of m*(|r|^2*I - r*r^T) plus each cube's own m/6 on the diagonal.
		float ixx = 0f, iyy = 0f, izz = 0f, ixy = 0f, ixz = 0f, iyz = 0f;
		for (int i = 0; i < count; i++)
		{
			float rx = cx[i] + 0.5f - CenterOfMass.X;
			float ry = cy[i] + 0.5f - CenterOfMass.Y;
			float rz = cz[i] + 0.5f - CenterOfMass.Z;
			float m = mass[i];
			float own = m / 6f;
			ixx += m * (ry * ry + rz * rz) + own;
			iyy += m * (rx * rx + rz * rz) + own;
			izz += m * (rx * rx + ry * ry) + own;
			ixy -= m * rx * ry;
			ixz -= m * rx * rz;
			iyz -= m * ry * rz;
		}
		PMat3 inertia = default(PMat3);
		inertia.M00 = ixx; inertia.M01 = ixy; inertia.M02 = ixz;
		inertia.M10 = ixy; inertia.M11 = iyy; inertia.M12 = iyz;
		inertia.M20 = ixz; inertia.M21 = iyz; inertia.M22 = izz;
		m_invInertiaLocal = inertia.Inverse();
		InertiaScale = Math.Max(0.001f, (ixx + iyy + izz) / 3f);

		// Collision samples: distinct block corners that touch at least one empty cell (the outer surface).
		HashSet<long> cells = new HashSet<long>();
		for (int i = 0; i < count; i++)
		{
			cells.Add(Key(cx[i], cy[i], cz[i]));
		}
		HashSet<long> corners = new HashSet<long>();
		List<PVec3> samples = new List<PVec3>();
		for (int i = 0; i < count; i++)
		{
			for (int dx = 0; dx <= 1; dx++)
			{
				for (int dy = 0; dy <= 1; dy++)
				{
					for (int dz = 0; dz <= 1; dz++)
					{
						int px = cx[i] + dx;
						int py = cy[i] + dy;
						int pz = cz[i] + dz;
						if (!corners.Add(Key(px, py, pz)))
						{
							continue;
						}
						bool allFilled = true;
						for (int a = 0; a <= 1 && allFilled; a++)
						{
							for (int b = 0; b <= 1 && allFilled; b++)
							{
								for (int c = 0; c <= 1; c++)
								{
									if (!cells.Contains(Key(px - 1 + a, py - 1 + b, pz - 1 + c)))
									{
										allFilled = false;
										break;
									}
								}
							}
						}
						if (!allFilled)
						{
							samples.Add(new PVec3(px - CenterOfMass.X, py - CenterOfMass.Y, pz - CenterOfMass.Z));
						}
					}
				}
			}
		}
		m_samples = samples.ToArray();
		m_cn = new PVec3[m_samples.Length];
		m_cr = new PVec3[m_samples.Length];
	}

	public int SampleCount { get { return m_samples.Length; } }

	// External forces act for the current Step() call only and must be re-applied every frame.
	public void AddForce(PVec3 force)
	{
		m_force = m_force + force;
		m_hasForce = true;
	}

	public void AddForceAtPoint(PVec3 force, PVec3 worldPoint)
	{
		m_force = m_force + force;
		m_torque = m_torque + PVec3.Cross(worldPoint - Position, force);
		m_hasForce = true;
	}

	public void AddTorque(PVec3 torque)
	{
		m_torque = m_torque + torque;
		m_hasForce = true;
	}

	public PVec3 LocalToWorld(PVec3 local)
	{
		return Position + Rotation.ToMat3().Mul(local);
	}

	public void Step(float dt, IPhysWorld world)
	{
		if (Sleeping)
		{
			return;
		}
		float remaining = dt;
		while (remaining > 1e-6f && !Sleeping)
		{
			float h = Math.Min(remaining, 1f / 90f);
			remaining -= h;
			SubStep(h, world);
		}
		m_force = default(PVec3);
		m_torque = default(PVec3);
		m_hasForce = false;
	}

	float EffMass(PVec3 dir, PVec3 r, PMat3 invI)
	{
		return m_invMass + PVec3.Dot(dir, PVec3.Cross(invI.Mul(PVec3.Cross(r, dir)), r));
	}

	void ApplyImpulse(PVec3 impulse, PVec3 r, PMat3 invI)
	{
		Velocity = Velocity + impulse * m_invMass;
		AngularVelocity = AngularVelocity + invI.Mul(PVec3.Cross(r, impulse));
	}

	void SubStep(float h, IPhysWorld world)
	{
		Age += h;
		Velocity = new PVec3(Velocity.X, Velocity.Y - Gravity * h, Velocity.Z);

		PMat3 R = Rotation.ToMat3();
		PMat3 invI = PMat3.Mul(PMat3.Mul(R, m_invInertiaLocal), R.Transpose());
		if (m_hasForce)
		{
			Velocity = Velocity + m_force * (m_invMass * h);
			AngularVelocity = AngularVelocity + invI.Mul(m_torque) * h;
		}

		// 1. Find contacts: every surface corner that is inside a solid cell is pushed out through the nearest free face.
		m_contactCount = 0;
		float pushPX = 0f, pushNX = 0f, pushPY = 0f, pushNY = 0f, pushPZ = 0f, pushNZ = 0f;
		for (int i = 0; i < m_samples.Length; i++)
		{
			PVec3 rr = R.Mul(m_samples[i]);
			PVec3 wp = Position + rr;
			int cx = (int)MathF.Floor(wp.X);
			int cy = (int)MathF.Floor(wp.Y);
			int cz = (int)MathF.Floor(wp.Z);
			if (!world.IsSolid(cx, cy, cz))
			{
				continue;
			}
			float fx = wp.X - cx;
			float fy = wp.Y - cy;
			float fz = wp.Z - cz;
			float best = float.MaxValue;
			int bestFace = -1;
			float bestAny = float.MaxValue;
			int anyFace = 3;
			for (int f = 0; f < 6; f++)
			{
				float d;
				int nx = 0, ny = 0, nz = 0;
				switch (f)
				{
					case 0: d = fx; nx = -1; break;
					case 1: d = 1f - fx; nx = 1; break;
					case 2: d = fy; ny = -1; break;
					case 3: d = 1f - fy; ny = 1; break;
					case 4: d = fz; nz = -1; break;
					default: d = 1f - fz; nz = 1; break;
				}
				if (d < bestAny)
				{
					bestAny = d;
					anyFace = f;
				}
				if (d < best && !world.IsSolid(cx + nx, cy + ny, cz + nz))
				{
					best = d;
					bestFace = f;
				}
			}
			if (bestFace < 0)
			{
				bestFace = anyFace;
				best = bestAny;
			}
			float depth = best + 0.001f;
			PVec3 n;
			switch (bestFace)
			{
				case 0: n = new PVec3(-1f, 0f, 0f); pushNX = Math.Max(pushNX, depth); break;
				case 1: n = new PVec3(1f, 0f, 0f); pushPX = Math.Max(pushPX, depth); break;
				case 2: n = new PVec3(0f, -1f, 0f); pushNY = Math.Max(pushNY, depth); break;
				case 3: n = new PVec3(0f, 1f, 0f); pushPY = Math.Max(pushPY, depth); break;
				case 4: n = new PVec3(0f, 0f, -1f); pushNZ = Math.Max(pushNZ, depth); break;
				default: n = new PVec3(0f, 0f, 1f); pushPZ = Math.Max(pushPZ, depth); break;
			}
			m_cn[m_contactCount] = n;
			m_cr[m_contactCount] = rr;
			m_contactCount++;
		}
		ContactCount = m_contactCount;

		// 2. Position correction (push out of the terrain, gently).
		if (m_contactCount > 0)
		{
			Position = new PVec3(
				Position.X + (pushPX - pushNX) * 0.8f,
				Position.Y + (pushPY - pushNY) * 0.8f,
				Position.Z + (pushPZ - pushNZ) * 0.8f);
		}

		// 3. Velocity constraints: contacts (normal + friction impulses) and the optional pin joint.
		PVec3 pinArm = PinEnabled ? R.Mul(PinLocal) : default(PVec3);
		for (int it = 0; it < 6; it++)
		{
			for (int c = 0; c < m_contactCount; c++)
			{
				PVec3 nrm = m_cn[c];
				PVec3 r = m_cr[c];
				PVec3 vp = Velocity + PVec3.Cross(AngularVelocity, r);
				float vn = PVec3.Dot(vp, nrm);
				if (vn >= 0f)
				{
					continue;
				}
				float e = (vn < -3f) ? Restitution : 0f;
				float j = -(1f + e) * vn / EffMass(nrm, r, invI);
				ApplyImpulse(nrm * j, r, invI);

				vp = Velocity + PVec3.Cross(AngularVelocity, r);
				PVec3 vt = vp - nrm * PVec3.Dot(vp, nrm);
				float vtl = vt.Length;
				if (vtl > 1e-4f)
				{
					PVec3 t = vt * (1f / vtl);
					float jt = Math.Min(vtl / EffMass(t, r, invI), Friction * j);
					ApplyImpulse(t * (-jt), r, invI);
				}
			}
			if (PinEnabled)
			{
				for (int axis = 0; axis < 3; axis++)
				{
					PVec3 e = (axis == 0) ? new PVec3(1f, 0f, 0f) : (axis == 1) ? new PVec3(0f, 1f, 0f) : new PVec3(0f, 0f, 1f);
					PVec3 vpp = Velocity + PVec3.Cross(AngularVelocity, pinArm);
					float vc = PVec3.Dot(vpp, e);
					float j = -vc / EffMass(e, pinArm, invI);
					ApplyImpulse(e * j, pinArm, invI);
				}
			}
		}

		// 4. Integrate.
		float speed = Velocity.Length;
		if (speed > MaxSpeed)
		{
			Velocity = Velocity * (MaxSpeed / speed);
		}
		Position = Position + Velocity * h;
		PQuat dq = PQuat.Mul(new PQuat(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z, 0f), Rotation);
		Rotation = new PQuat(
			Rotation.X + 0.5f * h * dq.X,
			Rotation.Y + 0.5f * h * dq.Y,
			Rotation.Z + 0.5f * h * dq.Z,
			Rotation.W + 0.5f * h * dq.W).Normalized();

		float angDamp = AngularDamping + ((m_contactCount > 0) ? ContactAngularDamping : 0f);
		Velocity = Velocity * Math.Max(0f, 1f - LinearDamping * h);
		AngularVelocity = AngularVelocity * Math.Max(0f, 1f - angDamp * h);

		if (PinEnabled)
		{
			PMat3 R2 = Rotation.ToMat3();
			PVec3 err = PinWorld - (Position + R2.Mul(PinLocal));
			Position = Position + err * 0.6f;
		}

		// 5. Sleep when resting.
		if (!PinEnabled && !NoSleep && m_contactCount > 0 && Velocity.Length < SleepSpeed && AngularVelocity.Length < SleepSpeed)
		{
			m_sleepTimer += h;
			if (m_sleepTimer >= SleepTime)
			{
				Sleeping = true;
				Velocity = default(PVec3);
				AngularVelocity = default(PVec3);
			}
		}
		else
		{
			m_sleepTimer = 0f;
		}
	}
}
