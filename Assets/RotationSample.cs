using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DefaultNamespace
{
    public class RotationSample : MonoBehaviour
    {
        [SerializeField] private Transform[] a;
        [SerializeField] private Transform[] b;

        public Transform aParent;


        private void Start()
        {
            var aMat = GeneratePositionMatrix(a);
            var bMat = GeneratePositionMatrix(b);

            var estimator = new PointSetsMatrixEstimator(aMat, bMat);
            var ans = estimator.GetTransformedPositionsMatrix();
            Debug.Log(ConvertToUnityMatrix(ans));
            for (int i = 0; i < a.Count(); i++)
            {
                a[i].position = new Vector3((float)ans[0, i], (float)ans[1, i], (float)ans[2, i]);
            }
            
            var (r1, r2) = estimator.GetTransformationMatrixes();
            Debug.Log(ConvertToUnityMatrix(r2));
            AdjustParent();
        }

        private void AdjustParent()
        {
            var tempG = Vector3.zero;
            foreach (var v in a)
            {
                tempG += v.position;
            }

            var g = tempG / a.Length;
            var x = a[0].position - g;
            var y = Vector3.Cross( a[1].position - g,x);
            var z = Vector3.Cross(y, x);
            aParent.transform.position = g;
            var newRotation = Quaternion.LookRotation(z, y);
            aParent.transform.rotation = newRotation;
        }

        private Matrix4x4 ConvertToUnityMatrix(double[,] rawMatrix)
        {
            var unityMat = new Matrix4x4();
            for (int i = 0; i < rawMatrix.GetLength(0); i++)
            {
                unityMat.SetRow(i,
                    new Vector4((float) rawMatrix[i, 0], (float) rawMatrix[i, 1], (float) rawMatrix[i, 2], 0));
            }

            unityMat.SetRow(3, new Vector4(0, 0, 0, 1));
            return unityMat;
        }

        private double[,] GeneratePositionMatrix(IEnumerable<Transform> transforms)
        {
            var tfList = transforms.ToList();
            var posMatrix = new double[3, tfList.Count()];
            for (int i = 0; i < tfList.Count(); i++)
            {
                var pos = tfList[i].position;
                posMatrix[0, i] = pos.x;
                posMatrix[1, i] = pos.y;
                posMatrix[2, i] = pos.z;
            }

            return posMatrix;
        }
    }
}