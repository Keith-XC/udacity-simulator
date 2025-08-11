using System.Collections.Generic;
using UnityEngine;

public class RoadMeshBuilder
{
    private GameObject roadPrefabMesh;
    private Texture2D customRoadTexture;
    private Texture2D[] roadTextures;
    private int iRoadTexture;
    private float roadWidth;
    private float roadOffsetW;
    private float roadHeightOffset;
    private TerrainToolkit terToolkit;
    private bool doFlattenArroundRoad;
    private bool doLiftRoadToTerrain;
    private Terrain terrain;
    private Texture2D normalRoadTexture;
    private Texture2D intersectionTexture;
    private bool useIntersectionTextureForIntersections;
    private GameObject endLine;
    private readonly List<Vector3> leftEdgeVertices = new List<Vector3>();
    private readonly List<Vector3> rightEdgeVertices = new List<Vector3>();

    /// <summary>
    /// Initializes a new instance of the RoadMeshBuilder class with the specified parameters.
    /// </summary>
    public RoadMeshBuilder(GameObject roadPrefab, Texture2D customTex, Texture2D[] textures, int textureIndex,
                           float rWidth, float rOffsetW, float rHeightOffset, TerrainToolkit toolkit, bool flattenAround, bool liftToTerrain, Terrain terrainObj,
                           Texture2D normalTex, Texture2D intersectTex, bool useIntersectTex, GameObject endLineObj)
    {
        roadPrefabMesh = roadPrefab;
        customRoadTexture = customTex;
        roadTextures = textures;
        iRoadTexture = textureIndex;
        roadWidth = rWidth;
        roadOffsetW = rOffsetW;
        roadHeightOffset = rHeightOffset;
        terToolkit = toolkit;
        doFlattenArroundRoad = flattenAround;
        doLiftRoadToTerrain = liftToTerrain;
        terrain = terrainObj;
        normalRoadTexture = normalTex;
        intersectionTexture = intersectTex;
        useIntersectionTextureForIntersections = useIntersectTex;
        endLine = endLineObj;
    }

    /// <summary>
    /// Initializes the road mesh based on the provided CarPath and returns the generated road GameObject.
    /// </summary>
    public GameObject InitRoad(CarPath path)
    {
        if (path == null)
        {
            Debug.LogWarning("Kein Pfad in RoadMeshBuilder.InitRoad");
            return null;
        }
        if (terToolkit != null)
        {
            terToolkit.Flatten();
            terToolkit.PerlinGenerator(1, 0.1f, 10, 0.5f);
        }
        GameObject roadGO = Object.Instantiate(roadPrefabMesh);
        roadGO.tag = "road_mesh";
        MeshRenderer mr = roadGO.GetComponent<MeshRenderer>();
        MeshFilter mf = roadGO.GetComponent<MeshFilter>();
        MeshCollider mc = roadGO.GetComponent<MeshCollider>();
        Mesh mesh = new Mesh();
        AssignRoadTexture(mr);
        leftEdgeVertices.Clear();
        rightEdgeVertices.Clear();
        path.centerNodes = new List<PathNode>();
        int numQuads = path.nodes.Count - 1;
        int numVerts = (numQuads + 1) * 2;
        Vector3[] vertices = new Vector3[numVerts];
        Vector2[] uv = new Vector2[numVerts];
        Vector3[] normals = new Vector3[numVerts];
        for (int i = 0; i < numVerts; i++)
            normals[i] = Vector3.up;
        BuildVertices(path, vertices, uv);
        List<int> triNormal, triIntersection;
        BuildSubMeshes(vertices, normals, uv, out triNormal, out triIntersection);
        mesh.vertices = vertices;
        mesh.subMeshCount = 2;
        mesh.SetTriangles(triNormal, 0);
        mesh.SetTriangles(triIntersection, 1);
        mesh.normals = normals;
        mesh.uv = uv;
        mesh.RecalculateBounds();
        mf.mesh = mesh;
        mc.sharedMesh = mesh;
        Material matNormal = new Material(Shader.Find("Standard"));
        if (normalRoadTexture != null)
            matNormal.mainTexture = normalRoadTexture;
        Material matIntersection = new Material(Shader.Find("Standard"));
        if (intersectionTexture != null)
            matIntersection.mainTexture = intersectionTexture;
        matIntersection.renderQueue = 2100;
        mr.materials = new Material[] { matNormal, matIntersection };
        return roadGO;
    }

    /// <summary>
    /// Builds the vertices and UV coordinates for the road mesh from the given CarPath.
    /// </summary>
    private void BuildVertices(CarPath path, Vector3[] vertices, Vector2[] uv)
    {
        int numQuads = path.nodes.Count - 1;
        int iNode = 0;
        for (int iVert = 0; iVert < (numQuads + 1) * 2; iVert += 2)
        {
            PathNode nodeA, nodeB;
            Vector3 posA, posB, vLength, vWidth;
            if (iNode + 1 < path.nodes.Count)
            {
                nodeA = path.nodes[iNode];
                nodeB = path.nodes[iNode + 1];
                posA = nodeA.pos;
                posB = nodeB.pos;
                vLength = posB - posA;
            }
            else
            {
                nodeA = path.nodes[iNode];
                nodeB = path.nodes[0];
                posA = nodeA.pos;
                posB = nodeB.pos;
                vLength = posB - posA;
            }
            if (terToolkit != null && doFlattenArroundRoad)
                terToolkit.FlattenArround(posA, 10.0f, 30.0f);
            if (doLiftRoadToTerrain && terrain != null)
                posA.y = terrain.SampleHeight(posA) + terrain.transform.position.y + 1.0f;
            posA.y += roadHeightOffset;
            vWidth = Vector3.Cross(vLength, Vector3.up);
            Vector3 leftPos = posA + vWidth.normalized * (roadWidth + roadOffsetW);
            Vector3 rightPos = posA - vWidth.normalized * (roadWidth - roadOffsetW);
            if (doLiftRoadToTerrain && terrain != null)
            {
                leftPos.y = terrain.SampleHeight(leftPos) + terrain.transform.position.y;
                rightPos.y = terrain.SampleHeight(rightPos) + terrain.transform.position.y;
            }
            leftEdgeVertices.Add(leftPos);
            rightEdgeVertices.Add(rightPos);
            PathNode centerNode = new PathNode { pos = (leftPos + rightPos) * 0.5f, rotation = nodeA.rotation };
            path.centerNodes.Add(centerNode);
            vertices[iVert] = leftPos;
            vertices[iVert + 1] = rightPos;
            uv[iVert] = new Vector2(0.2f * iNode, 0f);
            uv[iVert + 1] = new Vector2(0.2f * iNode, 1f);
            iNode++;
        }
    }

    /// <summary>
    /// Builds sub-mesh indices for normal and intersection areas based on the vertices.
    /// </summary>
    private void BuildSubMeshes(Vector3[] vertices, Vector3[] normals, Vector2[] uv,
                                  out List<int> triNormal, out List<int> triIntersection)
    {
        triNormal = new List<int>();
        triIntersection = new List<int>();
        int numQuads = (vertices.Length / 2) - 1;
        int iVertOffset = 0;
        for (int iQuad = 0; iQuad < numQuads; iQuad++)
        {
            bool leftInter = EdgeSegmentIntersects(leftEdgeVertices, iQuad);
            bool rightInter = EdgeSegmentIntersects(rightEdgeVertices, iQuad);
            bool quadIsIntersection = (leftInter || rightInter) && useIntersectionTextureForIntersections;
            int[] quadIndices = { 0 + iVertOffset, 2 + iVertOffset, 1 + iVertOffset,
                                   2 + iVertOffset, 3 + iVertOffset, 1 + iVertOffset };
            if (quadIsIntersection)
            {
                if (iVertOffset + 3 < vertices.Length)
                {
                    vertices[iVertOffset].y += 0.01f;
                    vertices[iVertOffset + 1].y += 0.01f;
                    vertices[iVertOffset + 2].y += 0.01f;
                    vertices[iVertOffset + 3].y += 0.01f;
                }
                triIntersection.AddRange(quadIndices);
            }
            else
            {
                triNormal.AddRange(quadIndices);
            }
            iVertOffset += 2;
        }
    }

    /// <summary>
    /// Checks if the edge segment at the specified index in the edge list intersects any other segment.
    /// </summary>
    private bool EdgeSegmentIntersects(List<Vector3> edgeList, int segIndex)
    {
        if (segIndex < 0 || segIndex >= edgeList.Count - 1) return false;
        Vector3 A = edgeList[segIndex];
        Vector3 B = edgeList[segIndex + 1];
        for (int i = 0; i < edgeList.Count - 1; i++)
        {
            if (Mathf.Abs(i - segIndex) < 2)
                continue;
            Vector3 C = edgeList[i];
            Vector3 D = edgeList[i + 1];
            if (LineSegmentsIntersect2D(A, B, C, D, out _))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Determines whether two 2D line segments intersect and returns the intersection point.
    /// </summary>
    private bool LineSegmentsIntersect2D(Vector3 A1, Vector3 A2, Vector3 B1, Vector3 B2, out Vector3 inter)
    {
        inter = Vector3.zero;
        Vector2 a1 = new Vector2(A1.x, A1.z);
        Vector2 a2 = new Vector2(A2.x, A2.z);
        Vector2 b1 = new Vector2(B1.x, B1.z);
        Vector2 b2 = new Vector2(B2.x, B2.z);
        Vector2 r = a2 - a1;
        Vector2 s = b2 - b1;
        float rxs = Cross(r, s);
        if (Mathf.Abs(rxs) < 1e-6f) return false;
        float t = Cross((b1 - a1), s) / rxs;
        float u = Cross((b1 - a1), r) / rxs;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            inter = A1 + t * (A2 - A1);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Assigns the road texture to the provided MeshRenderer.
    /// </summary>
    private void AssignRoadTexture(MeshRenderer mr)
    {
        if (mr == null) return;
        if (customRoadTexture != null)
        {
            mr.material.mainTexture = customRoadTexture;
            return;
        }
        if (roadTextures != null && iRoadTexture < roadTextures.Length)
        {
            Texture2D t = roadTextures[iRoadTexture];
            if (t != null)
                mr.material.mainTexture = t;
        }
    }

    /// <summary>
    /// Computes the 2D cross product of two vectors.
    /// </summary>
    private float Cross(Vector2 v1, Vector2 v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }
}
