
using UnityEngine;

public class Lidar3D : MonoBehaviour
{
    [Header("Horizontal")]
    [Range(1, 360)]
    public int horizontalResolution = 31;

    [Range(1, 360)]
    public float horizontalFOV = 120f;

    [Header("Vertical")]
    [Range(1, 90)]
    public int verticalResolution = 7;

    [Range(1, 180)]
    public float verticalFOV = 35f;

    [Header("Detection")]
    public float maxDistance = 30f;
    public LayerMask obstacleMask = ~0;

    [Header("Gizmos")]
    public bool showGizmos = true;
    public bool showOnlyHits = false;

    private float[] distances;
    private bool[] hits;

    public int RayCount =>
        horizontalResolution * verticalResolution;

    public float[] Distances => distances;
    public bool[] Hits => hits;

    void Awake()
    {
        Initialize();
    }

    void Initialize()
    {
        int count = RayCount;

        distances = new float[count];
        hits = new bool[count];
    }

    public void Scan()
    {
        if (distances == null || distances.Length != RayCount)
            Initialize();

        int index = 0;

        for (int v = 0; v < verticalResolution; v++)
        {
            float pitch = verticalResolution == 1
                ? 0f
                : Mathf.Lerp(
                    -verticalFOV / 2f,
                    verticalFOV / 2f,
                    v / (float)(verticalResolution - 1)
                );

            for (int h = 0; h < horizontalResolution; h++)
            {
                float yaw = horizontalResolution == 1
                    ? 0f
                    : Mathf.Lerp(
                        -horizontalFOV / 2f,
                        horizontalFOV / 2f,
                        h / (float)(horizontalResolution - 1)
                    );

                Vector3 direction =
                    Quaternion.AngleAxis(yaw, transform.up) *
                    Quaternion.AngleAxis(-pitch, transform.right) *
                    transform.forward;

                bool hit = Physics.Raycast(
                    transform.position,
                    direction,
                    out RaycastHit hitInfo,
                    maxDistance,
                    obstacleMask,
                    QueryTriggerInteraction.Ignore
                );

                hits[index] = hit;

                distances[index] = hit
                    ? hitInfo.distance / maxDistance
                    : 1f;

                index++;
            }
        }
    }

    public float[] GetObservations()
    {
        return distances;
    }


    void OnDrawGizmos()
    {
        if (!showGizmos)
            return;

        if (distances == null || hits == null)
            return;

        int expected = horizontalResolution * verticalResolution;
        int available = Mathf.Min(
            expected,
            distances.Length,
            hits.Length
        );

        int index = 0;

        for (int v = 0; v < verticalResolution; v++)
        {
            float pitch = verticalResolution == 1
                ? 0f
                : Mathf.Lerp(
                    -verticalFOV / 2f,
                    verticalFOV / 2f,
                    v / (float)(verticalResolution - 1)
                );

            for (int h = 0; h < horizontalResolution; h++)
            {
                if (index >= available)
                    return;

                float yaw = horizontalResolution == 1
                    ? 0f
                    : Mathf.Lerp(
                        -horizontalFOV / 2f,
                        horizontalFOV / 2f,
                        h / (float)(horizontalResolution - 1)
                    );

                Vector3 direction =
                    Quaternion.AngleAxis(yaw, transform.up) *
                    Quaternion.AngleAxis(-pitch, transform.right) *
                    transform.forward;

                bool hit = hits[index];

                if (!showOnlyHits || hit)
                {
                    Gizmos.color = hit ? Color.red : Color.green;

                    float distance = hit
                        ? distances[index] * maxDistance
                        : maxDistance;

                    Gizmos.DrawRay(
                        transform.position,
                        direction * distance
                    );
                }

                index++;
            }
        }
    }

}
