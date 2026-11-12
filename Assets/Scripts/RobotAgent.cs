
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Linq;
using System.Collections.Generic;

[System.Serializable]
public struct NavigationRoute
{
    public Vector3 origin;
    public Vector3 target;

    public NavigationRoute(Vector3 origin, Vector3 target)
    {
        this.origin = origin;
        this.target = target;
    }
}


public class RobotAgent : Agent
{
    [Header("References")]
    public Robot robotController;

    private Transform target;

    public float maxDistance = 450f;
    public float maxSpeed = 10f;
    public float maxAngularSpeed = 5f;

    private float previousDistance;
    private bool distanceInitialized = false;
    
    private GameObject targetMarker;
    private LineRenderer targetBeam;
    private LineRenderer[] targetRings;

    private const float MarkerHeight = 7f;
    private const float MarkerRadius = 1.5f;

    private NavigationRoute[] routes =
    {
        new NavigationRoute(
            new Vector3(383.252f, -16.52249f, 95.17547f),
            new Vector3(296.3f, -17.2f, 109.7f)
        ),
        new NavigationRoute(
            new Vector3(309.0458f, -16.11479f, 111.3884f),
            new Vector3(298.18f, 0.77f, 208.2f)
        ),
        new NavigationRoute(
            new Vector3(-5.62859f, -17.34035f, 53.44241f),
            new Vector3(230.3f, -16.92f, 90.4f)
        ),
        new NavigationRoute(
            new Vector3(-10.5f, -13.18f, 44.9f),
            new Vector3(-62.9f, -16.27f, 96.05f)
        ),
        new NavigationRoute(
            new Vector3(-5.75188f, -16.39041f, 81.8746f),
            new Vector3(134.8f, -59.8f, -195.5f)
        ),
        new NavigationRoute(
            new Vector3(-747.5172f, -1.36874f, 99.11747f),
            new Vector3(-549.6f, -9.6f, 34.6f)
        ),
        new NavigationRoute(
            new Vector3(-1321.629f, 11.7f, 420.8f),
            new Vector3(-1212f, 9.6f, 101f)
        ),
        new NavigationRoute(
            new Vector3(-1179.97f, 62.08345f, 739.4885f),
            new Vector3(-1109.2f, 96.09f, 322.1f)
        ),
        new NavigationRoute(
            new Vector3(-1047.788f, 78.40872f, 648.2389f),
            new Vector3(-1325.14f, 8.91f, 452.4f)
        ),
        new NavigationRoute(
            new Vector3(-1334.863f, 11.39713f, 78.94662f),
            new Vector3(-1494.2f, 19.5f, -219.8f)
        )
    };


    [SerializeField] private int stuckStepsLimit = 150;
    [SerializeField] private float minMovement = 0.1f;

    private int stuckSteps;
    private Vector3 lastPosition;

    private int episodeSteps;
    [SerializeField] private int maxEpisodeSteps = 5000;

    [Header("Sensors")]
    public Lidar3D[] lidars;

    private List<NavigationRoute> routeBag = new List<NavigationRoute>();
    private int routeIndex = 0;
    private NavigationRoute lastRoute;

    public override void Initialize()
    {
        if (robotController == null)
            robotController = GetComponent<Robot>();

        if (robotController == null || routes == null || routes.Length == 0)
        {
            Debug.LogError("Robot o rutas no asignadas.");
            enabled = false;
            return;
        }

        GameObject targetObject = new GameObject("RobotTarget");
        target = targetObject.transform;

        target.position = routes[0].target;

        CreateTargetMarker();
    }

    public override void OnEpisodeBegin()
    {
        episodeSteps = 0;
        NavigationRoute route = GetRandomRoute();

        robotController.SetAgentInput(0f, 0f);

        transform.SetPositionAndRotation(
            route.origin,
            Quaternion.Euler(0f, 0f, 0f)
        );
        robotController.Reset();

        target.position = route.target;

        previousDistance = GetDistanceXZ(transform.position, target.position);
        distanceInitialized = true;
        
        lastPosition = robotController.transform.position;
        stuckSteps = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target Direction
        Vector3 direction = target.position - transform.position;
        direction.y = 0f;
        Vector3 localDirection = transform.InverseTransformDirection(direction.normalized);
        sensor.AddObservation(localDirection);

        // Target Distance
        float distance = GetDistanceXZ(transform.position, target.position);
        
        sensor.AddObservation(Mathf.Clamp01(distance / maxDistance));

        // Local Orientation
        sensor.AddObservation(transform.up);

        // Local Velocity
        Vector3 velocity = transform.InverseTransformDirection(robotController.Velocity);
        sensor.AddObservation(velocity / maxSpeed);

        // Angular Velocity
        Vector3 angularVelocity = transform.InverseTransformDirection(robotController.AngularVelocity);
        sensor.AddObservation(angularVelocity / maxAngularSpeed);

        // lidars 3D
        foreach(Lidar3D lidar in lidars)
        {
            lidar.Scan();
            foreach (float distances in lidar.GetObservations())
            {
                sensor.AddObservation(distances);
            }
        }
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeSteps++;

        float steering = actions.ContinuousActions[0];
        float throttle = actions.ContinuousActions[1];
        robotController.SetAgentInput(steering, throttle);

        CalculateProgressReward();
        CheckTarget();
        CheckFall();
        CheckStuck();
        CheckTime();
    }

    private void CalculateProgressReward()
    {
        float upright = Vector3.Dot(
            transform.up,
            Vector3.up
        );

        float currentDistance = GetDistanceXZ(transform.position, target.position);

        if (!distanceInitialized)
        {
            previousDistance = currentDistance;
            distanceInitialized = true;
            return;
        }

        float progress = previousDistance - currentDistance;

        if (upright < 0.5f)
        {
            float penalty = Mathf.Lerp(0.01f, 0.03f,
                Mathf.InverseLerp(0.5f, -1f, upright));

            AddReward(-penalty);
        }
        else
        {
             AddReward(Mathf.Clamp(progress, -0.01f, 0.1f));
            float tiltPenalty = Mathf.InverseLerp(0.71f, 0.5f, upright);
            AddReward(-0.003f * tiltPenalty);
        }

        previousDistance = currentDistance;
    }

    private void CheckTarget()
    {
        float upright = Vector3.Dot(
            transform.up,
            Vector3.up
        );

        float distance = GetDistanceXZ(transform.position, target.position);

        if (distance < 3f && upright > 0.5f)
        {
            AddReward(5f);
            EndEpisode();
        }
    }

    private void CheckFall()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        bool groundDetected = Physics.Raycast(
            origin,
            Vector3.down,
            out RaycastHit hit,
            20f,
            LayerMask.GetMask("Mine"),
            QueryTriggerInteraction.Ignore
        );
        if (!groundDetected)
        {
            AddReward(-5f);
            EndEpisode();
        }
    }

    private void CheckStuck()
    {
        Vector3 currentPosition = robotController.transform.position;

        float movement = GetDistanceXZ(currentPosition, lastPosition);

        if (movement >= minMovement)
        {
            lastPosition = currentPosition;
            stuckSteps = 0;
        }
        else
        {
            stuckSteps++;

            if (stuckSteps >= stuckStepsLimit)
            {
                AddReward(-5f);
                EndEpisode();
            }
        }
    }

    private void CheckTime()
    {
        if (episodeSteps >= maxEpisodeSteps)
        {
            AddReward(-0.01f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.layer == LayerMask.NameToLayer("Obstacle"))
        {
            AddReward(-5f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;
        Vector2 movement = UnityEngine.InputSystem.InputSystem
                .actions["Move"].ReadValue<Vector2>();
        actions[0] = movement.x;
        actions[1] = movement.y;
    }


    private float GetDistanceXZ(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;

        return Vector3.Distance(a, b);
    }

    private void Update()
    {
        if (target == null || targetMarker == null)
            return;

        targetMarker.transform.position = target.position;

        float time = Time.time;

        targetBeam.positionCount = 2;
        targetBeam.SetPosition(0, Vector3.up * 0.2f);
        targetBeam.SetPosition(1, Vector3.up * MarkerHeight);

        for (int i = 0; i < targetRings.Length; i++)
        {
            float height = 0.2f + i * 0.12f;
            float radius = MarkerRadius + Mathf.Sin(time * 2f + i) * 0.15f;

            DrawRing(targetRings[i], radius, height);
        }

        Transform diamond = targetMarker.transform.Find("TargetDiamond");

        if (diamond != null)
        {
            diamond.localPosition = Vector3.up *
                (MarkerHeight + Mathf.Sin(time * 2f) * 0.4f);

            diamond.Rotate(Vector3.up * 90f * Time.deltaTime);
        }
    }

    private void DrawRing(LineRenderer line, float radius, float height)
    {
        for (int i = 0; i < 64; i++)
        {
            float angle = i * Mathf.PI * 2f / 64f;

            line.SetPosition(i, new Vector3(
                Mathf.Cos(angle) * radius,
                height,
                Mathf.Sin(angle) * radius
            ));
        }
    }

    private void CreateTargetMarker()
    {
        targetMarker = new GameObject("CrazyTaxi_TargetMarker");

        targetBeam = CreateLine(
            "TargetBeam",
            Color.yellow,
            0.25f,
            targetMarker.transform
        );

        targetRings = new LineRenderer[3];

        for (int i = 0; i < targetRings.Length; i++)
        {
            targetRings[i] = CreateLine(
                $"TargetRing_{i}",
                i == 1 ? Color.yellow : new Color(1f, 0.65f, 0f),
                0.12f,
                targetMarker.transform
            );
        }

        GameObject diamond = GameObject.CreatePrimitive(PrimitiveType.Cube);
        diamond.name = "TargetDiamond";
        diamond.transform.SetParent(targetMarker.transform);

        diamond.transform.localPosition = Vector3.up * MarkerHeight;
        diamond.transform.localScale = new Vector3(0.8f, 0.8f, 0.8f);
        diamond.transform.localRotation = Quaternion.Euler(45f, 45f, 0f);

        Destroy(diamond.GetComponent<Collider>());

        Shader shader = Shader.Find("Universal Render Pipeline/Lit");

        if (shader == null)
        {
            Debug.LogError("No se encontró el shader URP/Lit.");
            return;
        }

        Material material = new Material(shader);
        material.color = Color.yellow;

        if (material.HasProperty("_EmissionColor"))
        {
            material.EnableKeyword("_EMISSION");
            material.SetColor("_EmissionColor", Color.yellow * 2f);
        }

        diamond.GetComponent<Renderer>().material = material;
    }

    private LineRenderer CreateLine(
        string name,
        Color color,
        float width,
        Transform parent)
    {
        GameObject obj = new GameObject(name);
        obj.transform.SetParent(parent);

        LineRenderer line = obj.AddComponent<LineRenderer>();

        line.useWorldSpace = false;
        line.loop = true;
        line.positionCount = 64;
        line.startWidth = width;
        line.endWidth = width;

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");

        if (shader == null)
        {
            Debug.LogError("No se encontró URP/Unlit.");
            return line;
        }

        Material material = new Material(shader);
        material.color = color;

        line.material = material;
        line.startColor = color;
        line.endColor = color;

        return line;
    }

    private NavigationRoute GetRandomRoute()
    {
        if (routeIndex >= routeBag.Count)
        {
            routeBag = routes.ToList();

            for (int i = routeBag.Count - 1; i > 0; i--)
            {
                int j = Random.Range(0, i + 1);

                (routeBag[i], routeBag[j]) =
                    (routeBag[j], routeBag[i]);
            }

            if (routeBag.Count > 1 &&
    routeBag[0].origin == lastRoute.origin &&
    routeBag[0].target == lastRoute.target)
            {
                int j = Random.Range(1, routeBag.Count);

                (routeBag[0], routeBag[j]) =
                    (routeBag[j], routeBag[0]);
            }

            routeIndex = 0;
        }

        lastRoute = routeBag[routeIndex];

        return routeBag[routeIndex++];
    }
}