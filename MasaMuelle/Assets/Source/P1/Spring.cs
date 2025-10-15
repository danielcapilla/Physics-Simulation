using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public float Length0;
    public float Length;

    Vector3 dir;
    private PhysicsManager Manager;

    // Update is called once per frame
    void Update () {

        Vector3 yaxis = new Vector3(0.0f, 1.0f, 0.0f);
        dir = nodeA.Pos - nodeB.Pos;
        dir.Normalize();

        transform.position = 0.5f * (nodeA.Pos + nodeB.Pos);
        //The default length of a cylinder in Unity is 2.0
        transform.localScale = new Vector3(transform.localScale.x, Length / 2.0f, transform.localScale.z);
        transform.rotation = Quaternion.FromToRotation(yaxis, dir);
	}

    // Use this for initialization
    public void Initialize(PhysicsManager m)
    {
        Manager = m;

        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED //
        //Tiene que escribir la fuerza en la posición de los dos nodos 2*3
        //force[nodeA.index]
        Vector3 springForce = -Stiffness * (Length - Length0) * dir;
        force[nodeA.index] += springForce.x;
        force[nodeB.index] -= springForce.x;
        force[nodeA.index + 1] += springForce.y;
        force[nodeB.index + 1] -= springForce.y;
        force[nodeA.index + 2] += springForce.z;
        force[nodeB.index + 2] -= springForce.z;

    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx)
    {
        // TO BE COMPLETED //

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




    }

}
