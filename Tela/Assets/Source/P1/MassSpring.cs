using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using MathNet.Numerics.Distributions;
using System;
using UnityEditor.Experimental.GraphView;
using UnityEngine.UIElements;
using static PhysicsManager;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class MassSpring : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public MassSpring()
    {
        Manager = null;
    }

    #region EditorVariables

    public List<Node> Nodes;
    public List<Spring> Springs;

    public float Mass;
    public float StiffnessStretch;
    public float StiffnessBend;
    public float DampingAlpha;
    public float DampingBeta;

    private Mesh mesh;
    private Vector3[] vertices;
    private int[] triangles;

    #endregion

    #region OtherVariables
    private PhysicsManager Manager;

    private int index;
    private List<(int, int)> edges;
    private HashSet<(int, int)> edgeSet;
    private Dictionary<(int, int), List<int>> edgeToTriangles;
    #endregion

    #region MonoBehaviour

    public void Awake()
    {
        mesh = this.GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        triangles = mesh.triangles;

        // TO BE COMPLETED
        Manager = GetComponentInParent<PhysicsManager>();
        createTheNodes(vertices);
        createTheEdges(triangles);
        createTheSprings();


    }
    public void createTheNodes(Vector3[] vertices)
    {
        //Crea los nodos transformados a globales
        Nodes = new List<Node>();
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 pos = transform.TransformPoint(vertices[i]);
            Node newNode = new Node(pos);
            newNode.Initialize(i*3, Mass/vertices.Length, DampingAlpha, Manager);
            Nodes.Add(newNode);
        }
    }

    public void createTheEdges(int[] triangles)
    {
        // Rellenaas las aristas
        // Aristas sin duplicados
        edgeSet = new HashSet<(int, int)>(new EdgeComparer());
        // Aristas a triangulos
        edgeToTriangles = new Dictionary<(int, int), List<int>>(new EdgeComparer());

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];

            AddEdge(v1, v2, i / 3);
            AddEdge(v2, v3, i / 3);
            AddEdge(v3, v1, i / 3);
        }

        edges = new List<(int, int)>(edgeSet);
    }

    private void AddEdge(int v1, int v2, int triangleIndex)
    {
        //Agrega la arista 
        var edge = (Math.Min(v1, v2), Math.Max(v1, v2));
        edgeSet.Add(edge);
        // Comprueba si la arista esta ya en un triangulo
        if (!edgeToTriangles.ContainsKey(edge))
        {
            edgeToTriangles[edge] = new List<int>();
        }
        edgeToTriangles[edge].Add(triangleIndex);
    }

    public void createTheSprings()
    {
        Springs = new List<Spring>();
        HashSet<(int, int)> processedEdges = new HashSet<(int, int)>(new EdgeComparer());

        foreach (var edge in edges)
        {
            // No repetir muelle por semi-arista
            if (!processedEdges.Contains(edge))
            {
                Node nodeA = Nodes[edge.Item1];
                Node nodeB = Nodes[edge.Item2];
                //Muelle tipo stretch
                Spring newSpring = new Spring(nodeA, nodeB, Spring.SpringType.Stretch);
                newSpring.Initialize(StiffnessStretch, DampingBeta, Manager);
                Springs.Add(newSpring);
                processedEdges.Add(edge);
                // Si la arista tiene dos triangulos, creamos un muelle de tipo bend
                if (edgeToTriangles[edge].Count == 2)
                {
                    int triangle1 = edgeToTriangles[edge][0];
                    int triangle2 = edgeToTriangles[edge][1];

                    int oppositeVertex1 = GetOppositeVertex(triangle1, edge);
                    int oppositeVertex2 = GetOppositeVertex(triangle2, edge);

                    if (oppositeVertex1 != -1 && oppositeVertex2 != -1)
                    {
                        Node nodeC = Nodes[oppositeVertex1];
                        Node nodeD = Nodes[oppositeVertex2];
                        Spring bendSpring = new Spring(nodeC, nodeD, Spring.SpringType.Bend);
                        bendSpring.Initialize(StiffnessBend, DampingBeta, Manager);
                        Springs.Add(bendSpring);
                    }
                }
            }
        }
    }

    private int GetOppositeVertex(int triangleIndex, (int, int) edge)
    {
        // Obtener el vertice opuesto a la arista
        int v1 = triangles[triangleIndex * 3];
        int v2 = triangles[triangleIndex * 3 + 1];
        int v3 = triangles[triangleIndex * 3 + 2];

        if (v1 != edge.Item1 && v1 != edge.Item2) return v1;
        if (v2 != edge.Item1 && v2 != edge.Item2) return v2;
        if (v3 != edge.Item1 && v3 != edge.Item2) return v3;

        return -1;
    }
    public void Update()
    {
        // TO BE COMPLETED
        // Sincronizar parámetros del modelo con el estado actual
        foreach (var node in Nodes)
        {
            node.Mass = Mass / Nodes.Count; 
            node.Damping = DampingAlpha;
        }

        foreach (var spring in Springs)
    {
            if (spring.springType == Spring.SpringType.Stretch)
            {
                spring.Stiffness = StiffnessStretch;
                spring.Damping = DampingBeta;
            }
            else if (spring.springType == Spring.SpringType.Bend)
            {
                spring.Stiffness = StiffnessBend;
                spring.Damping = DampingBeta;
            }
        }
    }

    public void FixedUpdate()
    {
        // TO BE COMPLETED

        foreach (var spring in Springs)
        {
            spring.UpdateState();
        }
        for (int i = 0; i < Nodes.Count; i++)
        {
            vertices[i] = transform.InverseTransformPoint(Nodes[i].Pos);
        }
        mesh.vertices = vertices;
        mesh.RecalculateNormals();

    }
    public void OnDrawGizmos()
    {
        DebugDraw();
    }

    public void DebugDraw()
    {
        // Dibujar nodos
        foreach (var node in Nodes)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(node.Pos, 0.08f);
        }

        // Dibujar aristas y muelles
        foreach (var spring in Springs)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(spring.nodeA.Pos, spring.nodeB.Pos);
        }
    }
    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
    {
        // TO BE COMPLETED
        index = ind;
        Manager = m;

        // Fijar nodos según los fixers
        foreach (var fixer in fixers)
        {
            foreach (var node in Nodes)
            {
                if (fixer.IsInside(node.Pos))
                {
                    node.Fixed = true;
                }
            }
        }
        Debug.Log("MassSpring initialized");    
    }

    public int GetNumDoFs()
    {
        return 3 * Nodes.Count;
    }

    public void GetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetPosition(position);
    }

    public void SetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetPosition(position);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].UpdateState();
    }

    public void GetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetVelocity(velocity);
    }

    public void SetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetVelocity(velocity);
    }

    public void GetForce(VectorXD force)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForce(force);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForce(force);
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForceJacobian(dFdx, dFdv);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForceJacobian(dFdx, dFdv);
    }

    public void GetMass(MatrixXD mass)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMass(mass);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMassInverse(massInv);
    }

    public void FixVector(VectorXD v)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixVector(v);
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixMatrix(M);
        }
    }

    #endregion

    #region OtherMethods

    #region EdgeComparer
    private class EdgeComparer : IEqualityComparer<(int, int)>
    {
        public bool Equals((int, int) a, (int, int) b)
        {
            //Comparador de aristas
            return (a.Item1 == b.Item1 && a.Item2 == b.Item2) ||
                   (a.Item1 == b.Item2 && a.Item2 == b.Item1);
        }

        public int GetHashCode((int, int) edge)
        {
            //Generador de identificador único mismoa para (2,5) y (5,2)
            int hash1 = edge.Item1.GetHashCode();
            int hash2 = edge.Item2.GetHashCode();
            return hash1 ^ hash2;
        }
    }
    #endregion

    #endregion

}
