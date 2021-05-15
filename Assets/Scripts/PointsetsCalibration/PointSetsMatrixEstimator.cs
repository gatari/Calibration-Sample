using MathNet.Numerics.LinearAlgebra;

namespace PointsetsCalibration
{
    public class PointSetsMatrixEstimator
    {
        private readonly Matrix<double> _pointSetsB;
        private readonly Matrix<double> _pointSetsAd;
        private readonly Matrix<double> _translationMat;
        private readonly Matrix<double> _rotationMat;
        private readonly Matrix<double> _replacementMat;


        public PointSetsMatrixEstimator(double[,] pointSetsA, double[,] pointSetsB)
        {
            // 今回はAとBのサンプル数が同じな場合だけ
            var n = pointSetsA.GetLength(1);
            _translationMat = (1.0 / n) * Matrix<double>.Build.Dense(n, 1, 1.0) *
                              Matrix<double>.Build.Dense(1, n, 1.0);
            _pointSetsB = Matrix<double>.Build.DenseOfArray(pointSetsB);
            _pointSetsAd = Matrix<double>.Build.DenseOfArray(pointSetsA) *
                           (Matrix<double>.Build.DiagonalIdentity(n) - _translationMat);
            var pointSetsBd = _pointSetsB * (Matrix<double>.Build.DiagonalIdentity(n) - _translationMat);
            var svdA = _pointSetsAd.Svd(true);
            var svdB = pointSetsBd.Svd(true);
            _rotationMat = svdB.U * svdA.U.Transpose();
            _replacementMat = svdA.VT.Transpose() * svdB.VT;
        }

        public (double[,] rotationMat, double[,] replacementMat) GetTransformationMatrixes()
        {
            return (_rotationMat.ToArray(), _replacementMat.ToArray());
        }

        public double[,] GetTransformedPositionsMatrix()
        {
            var b = _rotationMat * _pointSetsAd * _replacementMat + _pointSetsB * _translationMat;
            return b.ToArray();
        }

        public double[,] GetRotatedPositionsMatrix()
        {
            var b = _rotationMat * _pointSetsAd * _replacementMat;
            return b.ToArray();
        }

        // public double[,] GetTransformedPosition(double x, double y, double z)
        // {
        //     var pos = new double[,]
        //     {
        //         {x},
        //         {y},
        //         {z}
        //     };
        //     var b = _rotationMat * Matrix<double>.Build.DenseOfArray(pos) + _pointSetsB * _translationMat.Column(0).ToColumnMatrix();
        //     return b.ToArray();
        // }
        //
        // public double[,] GetRotatedPosition(double x, double y, double z)
        // {
        //     var pos = new double[,]
        //     {
        //         {x},
        //         {y},
        //         {z}
        //     };
        //     var b = _rotationMat * Matrix<double>.Build.DenseOfArray(pos);
        //     return b.ToArray();
        // }
    }
}