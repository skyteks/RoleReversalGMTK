using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using UnityEngine.Events;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class Boid : MonoBehaviour
{
    public static bool useSeperationRule;
    public static bool useAllignmentRule;
    public static bool useCohesionRule;

    [SerializeField]
    [Range(0f, 20f)]
    private float viewRadius = 10f;
    [SerializeField]
    [Range(0f, 360f)]
    private float viewAngle = 270f;
    [SerializeField]
    private LayerMask obstacleMask = new LayerMask().ToEverything();
    [SerializeField]
    private float viewSphereCastRadius = 0.1f;

    private SphereCollider visionTrigger;
    private List<Boid> boidsInSight;

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        DebugDrawVision();
        //DebugDrawConnections();
        //DebugDrawDirections();
        //LookAround(DebugDrawLookPoints);
    }

    private void DebugDrawVision()
    {
        if (viewAngle > 0f && viewRadius > 0f)
        {
            Vector3 startVector = Quaternion.Euler(0f, -viewAngle * 0.5f, 0f) * transform.forward;

            Color color = Color.gray;
            color.a = 0.15f;
            Handles.color = color;
            Handles.DrawSolidArc(transform.position, Vector3.up, startVector, viewAngle, viewRadius);

            Handles.color = Color.yellow;
            //Handles.DrawWireArc(transform.position, Vector3.up, startVector, viewAngle, viewRadius);
            if (viewAngle < 360f)
            {
                Vector3 endVector = Quaternion.Euler(0f, viewAngle * 0.5f, 0f) * transform.forward;
                Vector3[] vectors = new Vector3[] { transform.position + startVector * viewRadius, transform.position, transform.position + endVector * viewRadius };
                //Handles.DrawLines(vectors, new int[] { 0, 1, 1, 2 });
            }
        }
    }

    private void DebugDrawConnections()
    {
        if (viewAngle > 0f && viewRadius > 0f && boidsInSight != null)
        {
            if (Application.isEditor && !Application.isPlaying)
            {
                visionTrigger = visionTrigger != null ? visionTrigger : GetComponent<SphereCollider>();
                boidsInSight ??= new List<Boid>();
                boidsInSight.Clear();
                Collider[] others = Physics.OverlapSphere(transform.TransformPoint(visionTrigger.center), viewRadius, new LayerMask().ToEverything(), QueryTriggerInteraction.Ignore);
                foreach (var other in others)
                {
                    OnTriggerEnter(other);
                }
            }

            Handles.color = Color.red;
            foreach (var other in boidsInSight)
            {
                Vector3 vector = other.transform.position - transform.position;
                float angle = Vector3.SignedAngle(transform.forward, vector, Vector3.up);
                if (Mathf.Abs(angle) >= viewAngle * 0.5f)
                {
                    continue;
                }
                Handles.DrawDottedLine(transform.position, other.transform.position, 3f);
            }
        }
    }

    private void DebugDrawDirections()
    {
        if (boidsInSight == null)
        {
            return;
        }
        const float lenght = 2f;
        Handles.color = Color.green;
        Handles.DrawLine(transform.position, transform.position + transform.forward * lenght);
        Handles.color = Color.blue;
        foreach (var other in boidsInSight)
        {
            Handles.DrawLine(other.transform.position, other.transform.position + transform.forward * lenght);
        }
    }

    private void DebugDrawLookPoints(Vector2 vector, float lerpT)
    {
        Handles.color = Color.Lerp(Color.red, Color.blue, lerpT);//Color.magenta;
        Handles.DrawSolidDisc(vector.ToVector3XZ(), Vector3.up, 0.02f);
    }
#endif

    void OnTriggerEnter(Collider other)
    {
        Boid neighbor;
        if (!other.gameObject.TryGetComponent<Boid>(out neighbor))
        {
            return;
        }

        if (neighbor == this)
        {
            return;
        }

        Vector3 vector = other.transform.position - transform.position;
        float angle = Vector3.SignedAngle(transform.forward, vector, Vector3.up);
        if (Mathf.Abs(angle) >= viewAngle * 0.5f)
        {
            return;
        }

        if (!boidsInSight.Contains(neighbor))
        {
            boidsInSight.Add(neighbor);
        }
    }

    void OnTriggerExit(Collider other)
    {
        Boid neighbor;
        if (!other.gameObject.TryGetComponent<Boid>(out neighbor))
        {
            return;
        }

        if (neighbor == this)
        {
            return;
        }

        boidsInSight.Remove(neighbor);
    }

    private Vector2[] LookAround(UnityAction<Vector2, float> callback)
    {
        int numPoints = 360;
        float angle = 360f / (float)numPoints;
        Vector2[] array = new Vector2[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            if (Mathf.Abs((angle * i) - 180f) < ((360f - viewAngle) * 0.5f))
            {
                continue;
            }

            float x = (angle * i).Sin();
            float y = (angle * i).Cos();
            Vector2 vector = new Vector2(x, y) * viewRadius;

            //float lerpT = i / (float)numPoints;
            float lerpT = Vector2.Distance(Vector2.up * viewRadius, vector) / viewRadius * 0.5f;
            //float lerpT = (-y + 1f) / 2f;

            callback?.Invoke(vector, lerpT);
            array[i] = vector;
        }
        return array;
    }
    /*
    private void LookAround_GoldenRatio(UnityAction<Vector2, float> callback)
    {
        int numPoints = 200;
        float pow = 0.5f;
        float turnFraction = 0.61803398875f; // golden ratio

        for (int i = 0; i < numPoints; i++)
        {
            float distance = Mathf.Pow(i / (numPoints - 1f), pow) * visionRadius;
            float angle = 2f * Mathf.PI * turnFraction * i;
            print(angle);

            float x = distance * Mathf.Cos(angle);
            float y = distance * Mathf.Sin(angle);
            Vector2 vector = new(x, y);

            //float lerpT = i / (float)numPoints);
            float lerpT = Vector2.Distance(Vector2.up * visionRadius, vector) / visionRadius * 0.5f;
            //float lerpT = (-y + visionRadius) / (visionRadius * 2f);

            callback.Invoke(vector, lerpT);
        }
    }
    */

    private Vector3 FindUnobstructedDirection()
    {
        Vector3 bestDir = transform.forward;
        float furthestUnobstructedDst = 0;
        RaycastHit hit;

        Vector2[] rayDirections = LookAround(null);

        for (int i = 0; i < rayDirections.Length; i++)
        {
            // tranform form local to world space so that smaller dir changes are examined first
            Vector3 dir = transform.TransformDirection(rayDirections[i].ToVector3XZ());
            if (Physics.SphereCast(transform.position, viewSphereCastRadius, dir, out hit, viewRadius, obstacleMask))
            {
                if (hit.distance > furthestUnobstructedDst)
                {
                    bestDir = dir;
                    furthestUnobstructedDst = hit.distance;
                    Debug.DrawRay(transform.position, bestDir, Color.red);
                }
            }
            else
            {
                // no obstacle in view radius, so return this direction
                Debug.DrawRay(transform.position, bestDir, Color.green);
                return dir;
            }
        }
        // obstacles all around, so return dir where obstacle is furthest away
        Debug.DrawRay(transform.position, bestDir, Color.green);
        return bestDir;
    }
}
