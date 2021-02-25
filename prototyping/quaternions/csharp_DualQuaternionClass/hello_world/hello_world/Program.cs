using System;
using System.Numerics;

namespace hello_world
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Program started");

            // Dual quaternion transformation
            DualQuaternion_c dq0 = new DualQuaternion_c(Quaternion.CreateFromYawPitchRoll(1, 2, 3), new Vector3(10, 30, 90));
            DualQuaternion_c dq1 = new DualQuaternion_c(Quaternion.CreateFromYawPitchRoll(-1, 3, 2), new Vector3(30, 40, 190));
            DualQuaternion_c dq2 = new DualQuaternion_c(Quaternion.CreateFromYawPitchRoll(2, 3, 1.5f), new Vector3(5, 20, 66));
            DualQuaternion_c dq_1 = dq0 * dq1 * dq2;
            DualQuaternion_c dq_2 = dq1 * dq2;
            Matrix4x4 dqToMatrix = DualQuaternion_c.DualQuaternionToMatrix(dq_1);

            // Matrix transformation
            Matrix4x4 m0 = Matrix4x4.CreateFromYawPitchRoll(1, 2, 3) * Matrix4x4.CreateTranslation(10, 30, 90);
            Matrix4x4 m1 = Matrix4x4.CreateFromYawPitchRoll(-1, 3, 2) * Matrix4x4.CreateTranslation(30, 40, 190);
            Matrix4x4 m2 = Matrix4x4.CreateFromYawPitchRoll(2, 3, 1.5f) * Matrix4x4.CreateTranslation(5, 20, 66);
            Matrix4x4 m = m0 * m1 * m2;

            Console.WriteLine(dqToMatrix);
            Console.WriteLine(m);

            Console.WriteLine("Program finished...");
        }
    }
}
