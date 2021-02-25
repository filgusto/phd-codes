using System.Numerics;
using System.Diagnostics;

namespace hello_world
{
    public class DualQuaternion_c
    {
        // Class parameters
        public Quaternion m_real;
        public Quaternion m_dual;

        // Constructors
        public DualQuaternion_c()
        {
            m_real = new Quaternion(0, 0, 0, 1);
            m_dual = new Quaternion(0, 0, 0, 0);
        }
        
        public DualQuaternion_c(Quaternion r, Quaternion d)
        {
            m_real = Quaternion.Normalize(r);
            m_dual = d;
        }

        public DualQuaternion_c(Quaternion r, Vector3 t)
        {
            m_real = Quaternion.Normalize(r);
            m_dual = (new Quaternion(t, 0) * m_real) * 0.5f;
        }


        // Common Methods
        public static float Dot( DualQuaternion_c a, DualQuaternion_c b)
        {
            return Quaternion.Dot(a.m_real, b.m_real);
        }

        public static DualQuaternion_c operator* (DualQuaternion_c q, float scale)
        {
            DualQuaternion_c ret = q;
            ret.m_real *= scale;
            ret.m_dual *= scale;
            return ret;
        }

        public static DualQuaternion_c Normalize (DualQuaternion_c q)
        {
            float mag = Quaternion.Dot(q.m_real, q.m_real);
            Debug.Assert(mag > 0.000001f);
            DualQuaternion_c ret = q;
            ret.m_real *= 1.0f / mag;
            ret.m_dual *= 1.0f / mag;
            return ret;
        }

        public static DualQuaternion_c operator + (DualQuaternion_c lhs, DualQuaternion_c rhs)
        {
            return new DualQuaternion_c(lhs.m_real + rhs.m_real, lhs.m_dual + rhs.m_dual);
        }

        // Multiplication order left to right
        public static DualQuaternion_c operator * (DualQuaternion_c lhs, DualQuaternion_c rhs)
        {
            return new DualQuaternion_c(rhs.m_real*lhs.m_real,
                                        rhs.m_dual*lhs.m_real + rhs.m_real*lhs.m_dual);
        }

        public static DualQuaternion_c Conjugate (DualQuaternion_c q)
        {
            return new DualQuaternion_c(Quaternion.Conjugate(q.m_real), Quaternion.Conjugate(q.m_dual));
        }

        public static Quaternion GetRotation (DualQuaternion_c q)
        {
            return q.m_real;
        }

        public static Vector3 GetTranslation (DualQuaternion_c q)
        {
            Quaternion t = (q.m_dual * 2.0f) * Quaternion.Conjugate(q.m_real);
            return new Vector3(t.X, t.Y, t.Z);
        }

        public static Matrix4x4 DualQuaternionToMatrix (DualQuaternion_c q)
        {
            q = DualQuaternion_c.Normalize(q);

            Matrix4x4 M = new Matrix4x4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);

            float w = q.m_real.W;
            float x = q.m_real.X;
            float y = q.m_real.Y;
            float z = q.m_real.Z;

            // Extract rotational information
            M.M11 = w * w + x * x - y * y - z * z;
            M.M12 = 2 * x * y + 2 * w * z;
            M.M13 = 2 * x * z - 2 * w * y;
            M.M21 = 2 * x * y - 2 * w * z;
            M.M22 = w * w + y * y - x * x - z * z;
            M.M23 = 2 * y * z + 2 * w * x;
            M.M31 = 2 * x * z + 2 * w * y;
            M.M32 = 2 * y * z - 2 * w * x;
            M.M33 = w * w + z * z - x * x - y * y;

            // Extract translation information
            Quaternion t = (q.m_dual * 2.0f) * Quaternion.Conjugate(q.m_real);
            M.M41 = t.X;
            M.M42 = t.Y;
            M.M43 = t.Z;

            return M;
        }


    }
}
