using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

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
		IntegrationMethod = Integration.Explicit;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
        Midpoint = 2,
        Verlet = 3,
        Implicit = 4,
	};

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public Integration IntegrationMethod;
    private bool first = true;
    private VectorXD x_prev = null;
    #endregion

    #region OtherVariables
    private List<ISimulable> m_objs;
    private int m_numDoFs;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Capacity);

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
            case Integration.Explicit: this.stepExplicit(); break;
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Midpoint: this.stepMidpoint(); break;
            case Integration.Verlet: this.stepVerlet(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Explicit integration.
    /// </summary>
    private void stepExplicit()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            //Le damos la posicion de todos los nodos y los guardamos en el objeto i
            // La X si es una masamuelle de 4 nodos X tiene tamaño 12
            //Si hay mas de un objeto, X guardara 3*numero de nodos de todos los objetos
            //ESCRIBE EN X el objeto i (sus X que le tocan)
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += TimeStep * v;
        v += TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            //Le damos la posicion de todos los nodos y los guardamos en el objeto i
            // La X si es una masamuelle de 4 nodos X tiene tamaño 12
            //Si hay mas de un objeto, X guardara 3*numero de nodos de todos los objetos
            //ESCRIBE EN X el objeto i (sus X que le tocan)
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        v += TimeStep * (Minv * f);
        x += TimeStep * v;
        //LE damos la posicion y la velocidad a todos los objetos que hemos recalculado
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
        //MEJOR CONSERVACIÓN DE LA ENERGÍA, MISMO ORDEN DE ERROR QUE EXPLICITO
    }

    /// <summary>
    /// Performs a simulation step using Midpoint integration.
    /// </summary>
    private void stepMidpoint()
    {
        // TO BE COMPLETED //
        //z(t+h/2) = z(t) + h/2 * f(z(t))
        //z(t+h) = z(t) + h * f(z(t+h/2))
        // Necesitamos la fuerza en el punto medio
        // Antes de llamar a GetForce(f), tengo que hacer un SetForce(f) en el punto medio

        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            //Le damos la posicion de todos los nodos y los guardamos en el objeto i
            // La X si es una masamuelle de 4 nodos X tiene tamaño 12
            //Si hay mas de un objeto, X guardara 3*numero de nodos de todos los objetos
            //ESCRIBE EN X el objeto i (sus X que le tocan)
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }
        //Calculamos en el punto intermedio (t+h/2)
        VectorXD v_mid = v + TimeStep/2.0 * (Minv * f);
        VectorXD x_mid = x + TimeStep/2.0 * v;
        //LE damos la posicion y la velocidad a todos los objetos que hemos recalculado EN EL PUNTO MEDIO
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x_mid);
            obj.SetVelocity(v_mid);
        }
        //Calculamos la fuerza en el punto medio
        //Limpiamos la fuerza puesto que vamos a calcular nuevas
        f.Clear();
        Minv.Clear();
        foreach (ISimulable obj in m_objs)
        {
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }
        x += TimeStep * v_mid;
        v += TimeStep * (Minv * f);
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Verlet integration.
    /// </summary>
    private void stepVerlet()
    {
        // TO BE COMPLETED //
        // Requiere guardar el estado anterior en el physics manager  Xi = Xi-1 -Xi-2
        // Inicializar x_prev con la posición inicial de todos los objetos
        // Requiere guardar el estado anterior en el physics manager
        
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        if (first)
        {
            // Inicializar x_prev con la posición inicial de todos los objetos
            x_prev = x + TimeStep * v + 0.5 * (Minv * f) * Mathf.Pow(TimeStep, 2);
            first = false;
            Debug.Log("First");
        }
        else
        {
            VectorXD x_new = 2.0 * x - x_prev + (Minv * f) * Mathf.Pow(TimeStep, 2);
            v = (x_new - x_prev) / (2.0 * TimeStep);
            x_prev = x;
            x = x_new;
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        // TO BE COMPLETED //
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        M.Clear();
        MatrixXD Jacobian = new DenseMatrixXD(m_numDoFs);
        Jacobian.Clear();

        foreach (ISimulable obj in m_objs)
        {
            //Le damos la posicion de todos los nodos y los guardamos en el objeto i
            // La X si es una masamuelle de 4 nodos X tiene tamaño 12
            //Si hay mas de un objeto, X guardara 3*numero de nodos de todos los objetos
            //ESCRIBE EN X el objeto i (sus X que le tocan)
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
            obj.GetMass(M);
            obj.GetForceJacobian(Jacobian);
        }
        // Aa * v = b
        //Hay que fijar A (df/dx)
        //A = I - h^2 * Minv * df/dx == A = M + h^2 * -df/dx
        //El implicito pierde energía, se va parando
        MatrixXD I = DenseMatrixXD.CreateIdentity(m_numDoFs);
        //MatrixXD A = M + Mathf.Pow(TimeStep, 2) * -Jacobian;
        MatrixXD A = I - Mathf.Pow(TimeStep, 2) * Minv  * Jacobian;
        //VectorXD b = M * v + TimeStep * f;
        VectorXD b = v + TimeStep * (Minv * f);
        foreach (ISimulable obj in m_objs)
        {
            obj.FixMatrix(A);
            obj.FixVector(b);
        }


        //A*v = b
        v = A.Solve(b);
        x += TimeStep * v;
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }

    }

}
