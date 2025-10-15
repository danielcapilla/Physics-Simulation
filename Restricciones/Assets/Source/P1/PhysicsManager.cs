using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using UnityEngine.UIElements;
using UnityEngine.Rendering.VirtualTexturing;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Symplectic;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Symplectic = 1,
        Implicit = 2,
        SymplecticConstraints = 3,
    };

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public List<GameObject> Constraints;
    public Integration IntegrationMethod;

    #endregion

    #region OtherVariables

    private List<ISimulable> m_objs;
    private List<IConstraint> m_constraints;
    private int m_numDoFs;
    private int m_numConstraints;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Count);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }

        //Parse the constraints
        m_numConstraints = 0;
        m_constraints = new List<IConstraint>(Constraints.Count);

        foreach (GameObject obj in Constraints)
        {
            IConstraint constraint = obj.GetComponent<IConstraint>();
            if (constraint != null)
            {
                m_constraints.Add(constraint);

                // Initialize constraint
                constraint.Initialize(m_numConstraints, this);

                // Retrieve the number of constraints
                m_numConstraints += constraint.GetNumConstraints();
            }
        }

    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            case Integration.SymplecticConstraints: this.stepSymplecticConstraints(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        // TO BE COMPLETED
        // No usar getPosition o SetPosition
        VectorXD velocities = new DenseVectorXD(m_numDoFs);
        VectorXD forces = new DenseVectorXD(m_numDoFs);
        VectorXD deltaX = new DenseVectorXD(m_numDoFs);
        forces.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(velocities);
            obj.GetForce(forces);
            obj.GetMassInverse(Minv);
           
        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(forces);
        }


        velocities += TimeStep * (Minv * forces);
        deltaX = TimeStep * velocities;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(deltaX);
            obj.SetVelocity(velocities);
        }


    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        // TO BE COMPLETED
        // No usar getPosition o SetPosition
        VectorXD velocities = new DenseVectorXD(m_numDoFs);
        VectorXD forces = new DenseVectorXD(m_numDoFs);
        VectorXD deltaX = new DenseVectorXD(m_numDoFs);
        forces.Clear();
        MatrixXD dFdx = new DenseMatrixXD(m_numDoFs);
        dFdx.Clear();
        MatrixXD dFdv = new DenseMatrixXD(m_numDoFs);
        dFdv.Clear();
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        M.Clear();
        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(velocities);
            obj.GetForce(forces);
            obj.GetMass(M);
            obj.GetForceJacobian(dFdx, dFdv);
        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(forces);
            constraint.GetForceJacobian(dFdx, dFdv);
        }
        MatrixXD A = M - TimeStep * dFdv - Mathf.Pow(TimeStep, 2) * dFdx;
        VectorXD b = (M - TimeStep * dFdv) * velocities + TimeStep * forces;
        //A*v = b
        velocities = A.Solve(b);
        deltaX = TimeStep * velocities;

        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(deltaX);
            obj.SetVelocity(velocities);
        }

    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration with constrained dynamics.
    /// The constraints are treated as implicit
    /// </summary>
    private void stepSymplecticConstraints()
    {
        // TO BE COMPLETED

        VectorXD velocities = new DenseVectorXD(m_numDoFs);
        VectorXD forces = new DenseVectorXD(m_numDoFs);
        VectorXD deltaX = new DenseVectorXD(m_numDoFs);
        VectorXD constraints = new DenseVectorXD(m_numConstraints);
        forces.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();
        // NConstraint x NDoFs
        MatrixXD dcdx = new DenseMatrixXD(m_numConstraints,m_numDoFs);
        dcdx.Clear();
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        M.Clear();
        MatrixXD dFdx = new DenseMatrixXD(m_numDoFs);
        dFdx.Clear();
        MatrixXD dFdv = new DenseMatrixXD(m_numDoFs);
        dFdv.Clear();
        foreach (ISimulable obj in m_objs)
        {
            obj.GetVelocity(velocities);
            obj.GetForce(forces);
            //obj.GetMassInverse(Minv);
            obj.GetForceJacobian(dFdx, dFdv);
            obj.GetMass(M);

        }
        foreach (IConstraint constraint in m_constraints)
        {
            constraint.GetForce(forces);
            constraint.GetConstraints(constraints);
            constraint.GetConstraintJacobian(dcdx);
        }
        // NDoFs x NDoFsS
        MatrixXD A = M;
        // NDoFs x 1
        VectorXD b = M * velocities + TimeStep * forces;

        // Resolvemos el sistema lineal para las velocidades
        MatrixXD Ainv = A.Inverse();
        // Calculamos el lado derecho de la ecuación
        VectorXD vRight = dcdx * Ainv * b + (1.0 / TimeStep) * constraints;
        // Calculamos el lado izquierdo de la ecuación
        MatrixXD vLeft = dcdx * Ainv * dcdx.Transpose();
        // Nconstraints x 1
        VectorXD lambda = vLeft.Solve(vRight);

        // Sin restricciones: V* = Ainv*b
        // Con restricciones: Frestriccion = -Jt*lamda  V = Ainv * (b - dcdx.Transpose() * lambda)
        velocities = Ainv * (b - dcdx.Transpose() * lambda);

        // Actualizar las posiciones: x(t + Δt) = x(t) + Δt * v(t + Δt)
        deltaX = TimeStep * velocities;


        // Establecer las nuevas posiciones y velocidades en los objetos simulables
        foreach (ISimulable obj in m_objs)
        {
            obj.AdvanceIncrementalPosition(deltaX);
            obj.SetVelocity(velocities);
        }
    }

}
