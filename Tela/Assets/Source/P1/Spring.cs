using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;


    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;
    }

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        // TO BE COMPLETED

        Manager = m;

        UpdateState();
        Length0 = Length;
        Stiffness = stiffness;
        Damping = damping;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED


        // Fuerza de resorte
        Vector3 springForce = -Stiffness * (Length - Length0) * dir;
        force[nodeA.index] += springForce.x;
        force[nodeB.index] -= springForce.x;
        force[nodeA.index + 1] += springForce.y;
        force[nodeB.index + 1] -= springForce.y;
        force[nodeA.index + 2] += springForce.z;
        force[nodeB.index + 2] -= springForce.z;

        // Fuerza de amortiguamiento
        //Fa = -(d*ut*(va-vb))*u
        //dm=beta*k
        Vector3 relativeVelocity = nodeA.Vel - nodeB.Vel;
        VectorXD rVelocityVector = DenseVectorXD.OfArray(new double[] { relativeVelocity.x, relativeVelocity.y, relativeVelocity.z });
        VectorXD uVector = DenseVectorXD.OfArray(new double[] { dir.x, dir.y, dir.z });
        MatrixXD uuT = uVector.ToColumnMatrix() * uVector.ToRowMatrix();
        // Const * 3x3 * 3x1 = 3x1
        VectorXD dampingForce = -(Damping*Stiffness) * uuT * rVelocityVector;
        force[nodeA.index] += dampingForce[0];
        force[nodeB.index] -= dampingForce[0];
        force[nodeA.index + 1] += dampingForce[1];
        force[nodeB.index + 1] -= dampingForce[1];
        force[nodeA.index + 2] += dampingForce[2];
        force[nodeB.index + 2] -= dampingForce[2];

    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED

        Vector3 u = (nodeA.Pos - nodeB.Pos) / Length;
        //Vector 1x3
        VectorXD uVector = DenseVectorXD.OfArray(new double[] { u.x, u.y, u.z });

        //Vector 3x1
        MatrixXD ut = uVector.ToRowMatrix();
        //Matriz 3x3
        MatrixXD uuT = uVector.ToColumnMatrix() * ut;
        MatrixXD I = MatrixXD.Build.DenseIdentity(3);
        MatrixXD dFaXa = -Stiffness * ((Length - Length0) / Length) * (I - uuT) - (Stiffness * uuT);
        MatrixXD dFbXb = dFaXa;
        MatrixXD dFbXa = -dFaXa;
        MatrixXD dFaXb = -dFaXa;
        //dFdv
        MatrixXD dFaVa = -(Damping*Stiffness) * uuT;
        MatrixXD dFbVb = dFaVa;
        MatrixXD dFaVb = -dFaVa;
        MatrixXD dFbVa = -dFaVa;
        //Rellenar la matriz dFdx
        //Nodo A
        dFdx[nodeA.index, nodeA.index] += dFaXa[0, 0];
        dFdx[nodeA.index, nodeA.index + 1] += dFaXa[0, 1];
        dFdx[nodeA.index, nodeA.index + 2] += dFaXa[0, 2];

        dFdx[nodeA.index + 1, nodeA.index] += dFaXa[1, 0];
        dFdx[nodeA.index + 1, nodeA.index + 1] += dFaXa[1, 1];
        dFdx[nodeA.index + 1, nodeA.index + 2] += dFaXa[1, 2];

        dFdx[nodeA.index + 2, nodeA.index] += dFaXa[2, 0];
        dFdx[nodeA.index + 2, nodeA.index + 1] += dFaXa[2, 1];
        dFdx[nodeA.index + 2, nodeA.index + 2] += dFaXa[2, 2];

        //Nodo B
        dFdx[nodeB.index, nodeB.index] += dFbXb[0, 0];
        dFdx[nodeB.index, nodeB.index + 1] += dFbXb[0, 1];
        dFdx[nodeB.index, nodeB.index + 2] += dFbXb[0, 2];

        dFdx[nodeB.index + 1, nodeB.index] += dFbXb[1, 0];
        dFdx[nodeB.index + 1, nodeB.index + 1] += dFbXb[1, 1];
        dFdx[nodeB.index + 1, nodeB.index + 2] += dFbXb[1, 2];

        dFdx[nodeB.index + 2, nodeB.index] += dFbXb[2, 0];
        dFdx[nodeB.index + 2, nodeB.index + 1] += dFbXb[2, 1];
        dFdx[nodeB.index + 2, nodeB.index + 2] += dFbXb[2, 2];

        //Nodo A y B
        dFdx[nodeA.index, nodeB.index] += dFaXb[0, 0];
        dFdx[nodeA.index, nodeB.index + 1] += dFaXb[0, 1];
        dFdx[nodeA.index, nodeB.index + 2] += dFaXb[0, 2];

        dFdx[nodeA.index + 1, nodeB.index] += dFaXb[1, 0];
        dFdx[nodeA.index + 1, nodeB.index + 1] += dFaXb[1, 1];
        dFdx[nodeA.index + 1, nodeB.index + 2] += dFaXb[1, 2];

        dFdx[nodeA.index + 2, nodeB.index] += dFaXb[2, 0];
        dFdx[nodeA.index + 2, nodeB.index + 1] += dFaXb[2, 1];
        dFdx[nodeA.index + 2, nodeB.index + 2] += dFaXb[2, 2];

        //Nodo B y A
        dFdx[nodeB.index, nodeA.index] += dFbXa[0, 0];
        dFdx[nodeB.index, nodeA.index + 1] += dFbXa[0, 1];
        dFdx[nodeB.index, nodeA.index + 2] += dFbXa[0, 2];

        dFdx[nodeB.index + 1, nodeA.index] += dFbXa[1, 0];
        dFdx[nodeB.index + 1, nodeA.index + 1] += dFbXa[1, 1];
        dFdx[nodeB.index + 1, nodeA.index + 2] += dFbXa[1, 2];

        dFdx[nodeB.index + 2, nodeA.index] += dFbXa[2, 0];
        dFdx[nodeB.index + 2, nodeA.index + 1] += dFbXa[2, 1];
        dFdx[nodeB.index + 2, nodeA.index + 2] += dFbXa[2, 2];

        //Rellenar la matriz dFdv
        //Nodo A
        dFdv[nodeA.index, nodeA.index] += dFaVa[0, 0];
        dFdv[nodeA.index, nodeA.index + 1] += dFaVa[0, 1];
        dFdv[nodeA.index, nodeA.index + 2] += dFaVa[0, 2];

        dFdv[nodeA.index + 1, nodeA.index] += dFaVa[1, 0];
        dFdv[nodeA.index + 1, nodeA.index + 1] += dFaVa[1, 1];
        dFdv[nodeA.index + 1, nodeA.index + 2] += dFaVa[1, 2];

        dFdv[nodeA.index + 2, nodeA.index] += dFaVa[2, 0];
        dFdv[nodeA.index + 2, nodeA.index + 1] += dFaVa[2, 1];
        dFdv[nodeA.index + 2, nodeA.index + 2] += dFaVa[2, 2];

        //Nodo B
        dFdv[nodeB.index, nodeB.index] += dFbVb[0, 0];
        dFdv[nodeB.index, nodeB.index + 1] += dFbVb[0, 1];
        dFdv[nodeB.index, nodeB.index + 2] += dFbVb[0, 2];

        dFdv[nodeB.index + 1, nodeB.index] += dFbVb[1, 0];
        dFdv[nodeB.index + 1, nodeB.index + 1] += dFbVb[1, 1];
        dFdv[nodeB.index + 1, nodeB.index + 2] += dFbVb[1, 2];

        dFdv[nodeB.index + 2, nodeB.index] += dFbVb[2, 0];
        dFdv[nodeB.index + 2, nodeB.index + 1] += dFbVb[2, 1];
        dFdv[nodeB.index + 2, nodeB.index + 2] += dFbVb[2, 2];

        //Nodo A y B
        dFdv[nodeA.index, nodeB.index] += dFaVb[0, 0];
        dFdv[nodeA.index, nodeB.index + 1] += dFaVb[0, 1];
        dFdv[nodeA.index, nodeB.index + 2] += dFaVb[0, 2];

        dFdv[nodeA.index + 1, nodeB.index] += dFaVb[1, 0];
        dFdv[nodeA.index + 1, nodeB.index + 1] += dFaVb[1, 1];
        dFdv[nodeA.index + 1, nodeB.index + 2] += dFaVb[1, 2];

        dFdv[nodeA.index + 2, nodeB.index] += dFaVb[2, 0];
        dFdv[nodeA.index + 2, nodeB.index + 1] += dFaVb[2, 1];
        dFdv[nodeA.index + 2, nodeB.index + 2] += dFaVb[2, 2];

        //Nodo B y A
        dFdv[nodeB.index, nodeA.index] += dFbVa[0, 0];
        dFdv[nodeB.index, nodeA.index + 1] += dFbVa[0, 1];
        dFdv[nodeB.index, nodeA.index + 2] += dFbVa[0, 2];

        dFdv[nodeB.index + 1, nodeA.index] += dFbVa[1, 0];
        dFdv[nodeB.index + 1, nodeA.index + 1] += dFbVa[1, 1];
        dFdv[nodeB.index + 1, nodeA.index + 2] += dFbVa[1, 2];

        dFdv[nodeB.index + 2, nodeA.index] += dFbVa[2, 0];
        dFdv[nodeB.index + 2, nodeA.index + 1] += dFbVa[2, 1];
        dFdv[nodeB.index + 2, nodeA.index + 2] += dFbVa[2, 2];

    }

}
