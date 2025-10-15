using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using static UnityEditor.ShaderData;

/// <summary>
/// Basic point constraint between two rigid bodies.
/// </summary>
public class PointConstraint : MonoBehaviour, IConstraint
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public PointConstraint()
    {
        Manager = null;
    }

    #region EditorVariables

    public float Stiffness;

    public RigidBody bodyA;
    public RigidBody bodyB;

    #endregion

    #region OtherVariables

    int index;
    private PhysicsManager Manager;

    protected Vector3 pointA;
    protected Vector3 pointB;

    Vector3 c;
    MatrixXD dcdx;

    Vector3 posA;
    Vector3 posB;

    #endregion

    #region MonoBehaviour

    // Update is called once per frame
    void Update()
    {
        // Compute the average position
         posA = (bodyA != null) ? bodyA.PointLocalToGlobal(pointA) : pointA;
         posB = (bodyB != null) ? bodyB.PointLocalToGlobal(pointB) : pointB;
        Vector3 pos = 0.5f * (posA + posB);
        c = posA - posB;
        // Apply the position
        Transform xform = GetComponent<Transform>();
        xform.position = pos;
    }

    #endregion

    #region IConstraint

    public void Initialize(int ind, PhysicsManager m)
    {
        index = ind;
        Manager = m;

        // Initialize local positions. We assume that the object is connected to a Sphere mesh.
        Transform xform = GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform to the constraint");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the constraint");
        }

        // Initialize kinematics
        Vector3 pos = xform.position;

        // Local positions on objects
        // Si el bodyA existe, guarda la posicion en local, si no, guarda la posicion en global
        pointA = (bodyA != null) ? bodyA.PointGlobalToLocal(pos) : pos;
        pointB = (bodyB != null) ? bodyB.PointGlobalToLocal(pos) : pos;

    }

    public int GetNumConstraints()
    {
        return 3;
    }

    public void GetConstraints(VectorXD c)
    {
        // TO BE COMPLETED

        if (bodyA == null && bodyB == null)
        {
            return;
        }


        // Escribir los valores de las restricciones en el vector de restricciones
        c.SetSubVector(index, 3, Utils.ToVectorXD(this.c));
    }

    public void GetConstraintJacobian(MatrixXD dcdx)
    {
        // TO BE COMPLETED
        // Rellenamos la matriz dcdx
        if (bodyA != null)
        {
            MatrixXD dcdxA = DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTa = -Utils.Skew(bodyA.m_rot * pointA);
            dcdx.SetSubMatrix(index, bodyA.index, dcdxA);
            dcdx.SetSubMatrix(index, bodyA.index + 3, dcdTa);
        }
        if (bodyB != null)
        {
            MatrixXD dcdxB = -DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTb = Utils.Skew(bodyB.m_rot * pointB);
            dcdx.SetSubMatrix(index, bodyB.index, dcdxB);
            dcdx.SetSubMatrix(index, bodyB.index+3, dcdTb);
        }

        this.dcdx = dcdx;
    }

    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED
        //C = xa + Ra*ra - xb - Rb*rb
        //Fxa = -k*C
        //Fxb = -Fxa
        //Ta = (Ra*ra)xFxa
        //Tb = (Rb*rb)xFxb
        //dcdxA = I
        //dcdxB = -I
        //dcdTa = (Ra*ra)xFxa
        //dcdTb = (Rb*rb)xFxb

        if (bodyA == null && bodyB == null)
        {
            return;
        }
        //c = bodyA.m_pos + bodyA.m_rot * pointA - bodyB.m_pos - bodyB.m_rot * pointB;

        //c = posA - posB;

        // Fuerzas de transalacion
        Vector3 FxaV = -Stiffness * c;
        VectorXD Fxa = Utils.ToVectorXD(FxaV);
        Vector3 FxbV = - FxaV;
        VectorXD Fxb = Utils.ToVectorXD(FxbV);

        // Asignar las fuerzas de restricción al vector de fuerzas global
        if (bodyA != null)
        {
            // 3 Dofs de traslación
            force[bodyA.index] += FxaV.x;
            force[bodyA.index + 1] += FxaV.y;
            force[bodyA.index + 2] += FxaV.z;

            // 3 Dofs de rotación
            Vector3 Ta = Vector3.Cross(bodyA.m_rot * pointA, FxaV);
            force[bodyA.index + 3] += Ta.x;
            force[bodyA.index + 4] += Ta.y;
            force[bodyA.index + 5] += Ta.z;
        }
        if (bodyB != null)
        {
            // 3 Dofs de traslación
            force[bodyB.index] += FxbV.x;
            force[bodyB.index + 1] += FxbV.y;
            force[bodyB.index + 2] += FxbV.z;

            // 3 Dofs de rotación
            Vector3 Tb = Vector3.Cross(bodyB.m_rot * pointB, FxbV);
            force[bodyB.index + 3] += Tb.x;
            force[bodyB.index + 4] += Tb.y;
            force[bodyB.index + 5] += Tb.z;
        }


    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        // Matrices 12x12
        // Para un cuerpo 6x6
        // |fA*3  thetaA*3 xB*3  thetaB*3
        // |tA*3 
        // |fB*3
        // |tB*3

        // |faXa  faXa faXa faTa faTa  faTa faXb  faXb faXb faTb  faTb faTb|
        // |faXa  faXa faXa faTa faTa  faTa faXb  faXb faXb faTb  faTb faTb|
        // |faXa  faXa faXa faTa faTa  faTa faXb  faXb faXb faTb  faTb faTb|
        // |taXa  taXa taXa taTa taTa  taTa taXb  taXb taXb taTb  taTb taTb|               
        // |taXa  taXa taXa taTa taTa  taTa taXb  taXb taXb taTb  taTb taTb|
        // |taXa  taXa taXa taTa taTa  taTa taXb  taXb taXb taTb  taTb taTb|

        if (bodyA != null)
        {

            MatrixXD dcdXa = DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTa = -Utils.Skew(bodyA.m_rot * pointA);
            MatrixXD Faxa = dcdXa.Transpose()*dcdXa;
            MatrixXD FaTa = dcdXa.Transpose() * dcdTa;
            MatrixXD TaXa = dcdTa.Transpose()*dcdXa;
            MatrixXD TaTa = dcdTa.Transpose() * dcdTa;

            //// FaXa
            //dFdx.SetSubMatrix(bodyA.index, bodyA.index, dFdx.SubMatrix(bodyA.index, 3, bodyA.index, 3) - Stiffness * DenseMatrixXD.CreateIdentity(3));
            //// FaTa
            //dFdx.SetSubMatrix(bodyA.index, bodyA.index + 3, dFdx.SubMatrix(bodyA.index, 3, bodyA.index + 3, 3) + Stiffness * Utils.Skew(bodyA.m_rot * pointA));
            //// TaXa
            //dFdx.SetSubMatrix(bodyA.index + 3, bodyA.index, dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index, 3) - Stiffness * Utils.Skew(bodyA.m_rot * pointA));
            //// TaTa
            //dFdx.SetSubMatrix(bodyA.index + 3, bodyA.index + 3, dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index + 3, 3) + Stiffness * Utils.Skew(bodyA.m_rot * pointA) * Utils.Skew(bodyA.m_rot * pointA));


            // FaXa
            dFdx.SetSubMatrix(bodyA.index, bodyA.index, dFdx.SubMatrix(bodyA.index, 3, bodyA.index, 3) - Stiffness * Faxa);
            // FaTa
            dFdx.SetSubMatrix(bodyA.index, bodyA.index + 3, dFdx.SubMatrix(bodyA.index, 3, bodyA.index + 3, 3) - Stiffness * FaTa);
            // TaXa
            dFdx.SetSubMatrix(bodyA.index + 3, bodyA.index, dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index, 3) - Stiffness * TaXa);
            // TaTa
            dFdx.SetSubMatrix(bodyA.index + 3, bodyA.index + 3, dFdx.SubMatrix(bodyA.index + 3, 3, bodyA.index + 3, 3) - Stiffness * TaTa);
        }
        if (bodyB != null)
        {
            MatrixXD dcdXb = -DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTb = Utils.Skew(bodyB.m_rot * pointB);
            MatrixXD FbxB = dcdXb.Transpose() * dcdXb;
            MatrixXD FbTb = dcdXb.Transpose() * dcdTb;
            MatrixXD TbXb = dcdTb.Transpose() * dcdXb;
            MatrixXD TbTb = dcdTb.Transpose() * dcdTb;

            //// FbXb
            //dFdx.SetSubMatrix(bodyB.index, bodyB.index, dFdx.SubMatrix(bodyB.index, 3, bodyB.index, 3) - Stiffness * DenseMatrixXD.CreateIdentity(3));
            //// FbTb
            //dFdx.SetSubMatrix(bodyB.index, bodyB.index + 3, dFdx.SubMatrix(bodyB.index, 3, bodyB.index + 3, 3) + Stiffness * Utils.Skew(bodyB.m_rot * pointB));
            //// TbXb
            //dFdx.SetSubMatrix(bodyB.index + 3, bodyB.index, dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index, 3) - Stiffness * Utils.Skew(bodyB.m_rot * pointB));
            //// TbTb
            //dFdx.SetSubMatrix(bodyB.index + 3, bodyB.index + 3, dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index + 3, 3) + Stiffness * Utils.Skew(bodyB.m_rot * pointB) * Utils.Skew(bodyB.m_rot * pointB));

            // FbXb
            dFdx.SetSubMatrix(bodyB.index, bodyB.index, dFdx.SubMatrix(bodyB.index, 3, bodyB.index, 3) - Stiffness * FbxB);
            // FbTb
            dFdx.SetSubMatrix(bodyB.index, bodyB.index + 3, dFdx.SubMatrix(bodyB.index, 3, bodyB.index + 3, 3) - Stiffness * FbTb);
            // TbXb
            dFdx.SetSubMatrix(bodyB.index + 3, bodyB.index, dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index, 3) - Stiffness * TbXb);
            // TbTb
            dFdx.SetSubMatrix(bodyB.index + 3, bodyB.index + 3, dFdx.SubMatrix(bodyB.index + 3, 3, bodyB.index + 3, 3) - Stiffness * TbTb);
        }
    
        if (bodyA != null && bodyB != null)
        {
            MatrixXD dcdXa = DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTa = -Utils.Skew(bodyA.m_rot * pointA);
            MatrixXD dcdXb = -DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTb = Utils.Skew(bodyB.m_rot * pointB);
            MatrixXD FaXb = dcdXa.Transpose() * dcdXb;
            MatrixXD FaTb = dcdXa.Transpose() * dcdTb;
            MatrixXD TaXb = dcdTa.Transpose() * dcdXb;
            MatrixXD TaTb = dcdTa.Transpose() * dcdTb;


            // FaXb
            dFdx.SetSubMatrix(bodyA.index, bodyB.index, dFdx.SubMatrix(bodyA.index, 3, bodyB.index, 3) - Stiffness *FaXb);
            // FaTb
            dFdx.SetSubMatrix(bodyA.index, bodyB.index + 3, dFdx.SubMatrix(bodyA.index, 3, bodyB.index + 3, 3) - Stiffness * FaTb);
            // TaXb
            dFdx.SetSubMatrix(bodyA.index + 3, bodyB.index, dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index, 3) - Stiffness * TaXb);
            // TaTb
            dFdx.SetSubMatrix(bodyA.index + 3, bodyB.index + 3, dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index + 3, 3) - Stiffness * TaTb);

            //// FaXb
            //dFdx.SetSubMatrix(bodyA.index, bodyB.index, dFdx.SubMatrix(bodyA.index, 3, bodyB.index, 3) + Stiffness * DenseMatrixXD.CreateIdentity(3));
            //// FaTb
            //dFdx.SetSubMatrix(bodyA.index, bodyB.index + 3, dFdx.SubMatrix(bodyA.index, 3, bodyB.index + 3, 3) - Stiffness * Utils.Skew(bodyB.m_rot * pointB));
            //// TaXb
            //dFdx.SetSubMatrix(bodyA.index + 3, bodyB.index, dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index, 3) + Stiffness * Utils.Skew(bodyA.m_rot * pointA));
            //// TaTb
            //dFdx.SetSubMatrix(bodyA.index + 3, bodyB.index + 3, dFdx.SubMatrix(bodyA.index + 3, 3, bodyB.index + 3, 3) - Stiffness * Utils.Skew(bodyA.m_rot * pointA) * Utils.Skew(bodyB.m_rot * pointB));
        }
        if (bodyA != null && bodyB != null)
        {

            MatrixXD dcdXa = DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTa = -Utils.Skew(bodyA.m_rot * pointA);
            MatrixXD dcdXb = -DenseMatrixXD.CreateIdentity(3);
            MatrixXD dcdTb = Utils.Skew(bodyB.m_rot * pointB);
            MatrixXD FbXa = dcdXb.Transpose() * dcdXa;
            MatrixXD FbTa = dcdXb.Transpose() * dcdTa;
            MatrixXD TbXa = dcdTb.Transpose() * dcdXa;
            MatrixXD TbTa = dcdTb.Transpose() * dcdTa;

            // FbXa
            dFdx.SetSubMatrix(bodyB.index, bodyA.index, dFdx.SubMatrix(bodyB.index, 3, bodyA.index, 3) - Stiffness * FbXa);
            // FbTa
            dFdx.SetSubMatrix(bodyB.index, bodyA.index + 3, dFdx.SubMatrix(bodyB.index, 3, bodyA.index + 3, 3) - Stiffness *FbTa);
            // TbXa
            dFdx.SetSubMatrix(bodyB.index + 3, bodyA.index, dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index, 3) - Stiffness * TbXa);
            // TbTa
            dFdx.SetSubMatrix(bodyB.index + 3, bodyA.index + 3, dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index + 3, 3) - Stiffness * TbTa);

            //// FbXa
            //dFdx.SetSubMatrix(bodyB.index, bodyA.index, dFdx.SubMatrix(bodyB.index, 3, bodyA.index, 3) + Stiffness * DenseMatrixXD.CreateIdentity(3));
            //// FbTa
            //dFdx.SetSubMatrix(bodyB.index, bodyA.index + 3, dFdx.SubMatrix(bodyB.index, 3, bodyA.index + 3, 3) - Stiffness * Utils.Skew(bodyA.m_rot * pointA));
            //// TbXa
            //dFdx.SetSubMatrix(bodyB.index + 3, bodyA.index, dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index, 3) + Stiffness * Utils.Skew(bodyB.m_rot * pointB));
            //// TbTa
            //dFdx.SetSubMatrix(bodyB.index + 3, bodyA.index + 3, dFdx.SubMatrix(bodyB.index + 3, 3, bodyA.index + 3, 3) - Stiffness * Utils.Skew(bodyB.m_rot * pointB) * Utils.Skew(bodyA.m_rot * pointA));
        }
    }

    #endregion

}
