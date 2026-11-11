using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

public class BallBehaviour : MonoBehaviour
{
    public enum Integrator { Euler, SemiImplicitEuler, RK4 }
    public enum HitType { Float, Topspin, Backspin, SideLeft, SideRight }
    private enum Phase { Ready, Flying, Landed }

    [Header("Aiming")]
    public PlayerCamera playerCamera;
    public Vector3 hitDirection = new Vector3(0f, 0.3f, 1f);

    [Header("Hit settings")]
    public HitType hitType = HitType.Topspin;
    [FormerlySerializedAs("hitForce")]
    public float hitSpeed = 30f;
    public float spinRate = 40f;
    public float floatSpinRate = 2f;
    [Range(-90f, 90f)]
    public float sideSpinTilt = 45f;

    [Header("Integration")]
    public Integrator integrator = Integrator.RK4;

    [Header("Ball properties")]
    public float mass = 0.27f;
    public float radius = 0.105f;

    [Header("Drag")]
    public float dragCoefficient = 0.17f;
    public bool useDragCrisis = false;
    public float subcriticalDragCoefficient = 0.45f;
    public float criticalReynolds = 220000f;
    public float criticalReynoldsWidth = 80000f;

    [Header("Air")]
    public float airDensity = 1.2f;
    public float dynamicViscosity = 1.81e-5f;

    [Header("Magnus effect")]
    public float liftSlope = 0.6f;
    public float maxLiftCoefficient = 0.3f;
    public float spinDecayRate = 0.03f;

    [Header("Float-serve instability (phenomenological, not validated)")]
    public float wobbleCoefficient = 0.05f;
    public float wobbleFrequency = 4f;
    public float floatSpinThreshold = 8f;

    [Header("Wind")]
    public bool windEnabled = false;
    public Vector3 baseWind = new Vector3(2f, 0f, 0f);
    public float gustStrength = 1f;
    public float gustFrequency = 0.3f;

    [Header("Visuals")]
    public Transform visualModel;
    public float spinVisualScale = 1f;

    [Header("Aim preview")]
    public bool showTrajectoryPreview = true;
    public Color previewColor = new Color(1f, 1f, 1f, 0.9f);
    public Color previewNetColor = new Color(1f, 0.35f, 0.35f, 0.9f);
    public float landingMarkerRadius = 0.35f;

    [Header("Court")]
    public float groundHeight = 0f;
    public Transform net;
    public float netHeight = 2.43f;
    public float netHalfWidth = 4.75f;
    public float resetDelay = 1.5f;

    private struct BallState
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 Spin;

        public BallState(Vector3 position, Vector3 velocity, Vector3 spin)
        {
            Position = position;
            Velocity = velocity;
            Spin = spin;
        }

        public static BallState operator +(BallState a, BallState b) =>
            new BallState(a.Position + b.Position, a.Velocity + b.Velocity, a.Spin + b.Spin);

        public static BallState operator *(BallState s, float k) =>
            new BallState(s.Position * k, s.Velocity * k, s.Spin * k);
    }

    private struct AirEnvironment
    {
        public Vector3 Wind;
        public Vector2 WobbleNoise;
    }

    private struct ForceSet
    {
        public Vector3 Gravity, Drag, Magnus, Wobble;
        public Vector3 Total => Gravity + Drag + Magnus + Wobble;
    }

    private class SimResult
    {
        public List<Vector3> Positions = new List<Vector3>();
        public Vector3 Landing;
        public float FlightTime;
        public int Steps;
    }

    private Phase phase = Phase.Ready;
    private BallState state;
    private Vector3 startPosition;
    private float flightTime, landedTimer, apexHeight, noiseSeed;
    private ForceSet lastForces;
    private string lastResult = "";
    private readonly StringBuilder flightLog = new StringBuilder();
    private TrailRenderer trail;
    private Quaternion visualStartRotation;
    private Vector3 prevPhysicsPosition;

    private Transform VisualTarget { get { return visualModel != null ? visualModel : transform; } }

    void Awake()
    {
        startPosition = transform.position;
        visualStartRotation = VisualTarget.localRotation;
        state = new BallState(startPosition, Vector3.zero, Vector3.zero);
        prevPhysicsPosition = startPosition;
        EnsureTrail();
        EnsurePreview();
    }

    void OnDestroy()
    {
        if (previewLine != null) Destroy(previewLine.gameObject);
        if (landingMarker != null) Destroy(landingMarker.gameObject);
    }

    private LineRenderer previewLine;
    private LineRenderer landingMarker;

    private void EnsurePreview()
    {
        previewLine = CreateLine("TrajectoryPreview", true);
        previewLine.startWidth = 0.04f;
        previewLine.endWidth = 0.015f;

        landingMarker = CreateLine("LandingMarker", false);
        landingMarker.loop = true;
        landingMarker.startWidth = 0.05f;
        landingMarker.endWidth = 0.05f;
        landingMarker.positionCount = 32;
        for (int i = 0; i < 32; i++)
        {
            float a = i / 32f * Mathf.PI * 2f;
            landingMarker.SetPosition(i, new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a)) * landingMarkerRadius);
        }
    }

    private LineRenderer CreateLine(string objectName, bool worldSpace)
    {
        GameObject go = new GameObject(objectName);
        LineRenderer lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = worldSpace;
        lr.material = new Material(Shader.Find("Sprites/Default"));
        lr.enabled = false;
        return lr;
    }

    private void UpdateTrajectoryPreview()
    {
        bool visible = showTrajectoryPreview && phase == Phase.Ready;
        previewLine.enabled = visible;
        landingMarker.enabled = visible;
        if (!visible) return;

        BallState initial = BuildInitialState(GetAimDirection(), hitType, hitSpeed);
        AirEnvironment env = new AirEnvironment
        {
            Wind = windEnabled ? baseWind : Vector3.zero,
            WobbleNoise = Vector2.zero
        };
        SimResult result = Simulate(Integrator.RK4, initial, 0.02f, env, true);

        List<Vector3> points = result.Positions;
        Vector3 end = result.Landing;
        bool hitsNet = false;

        for (int i = 1; i < points.Count; i++)
        {
            if (CrossesNet(points[i - 1], points[i], out Vector3 netPoint))
            {
                points = points.GetRange(0, i);
                points.Add(netPoint);
                end = netPoint;
                hitsNet = true;
                break;
            }
        }
        if (!hitsNet) points.Add(end);

        Color c = hitsNet ? previewNetColor : previewColor;
        previewLine.startColor = c;
        previewLine.endColor = new Color(c.r, c.g, c.b, c.a * 0.4f);
        previewLine.positionCount = points.Count;
        previewLine.SetPositions(points.ToArray());

        landingMarker.startColor = c;
        landingMarker.endColor = c;
        landingMarker.transform.position = hitsNet
            ? end
            : new Vector3(end.x, groundHeight + 0.02f, end.z);
    }

    private void EnsureTrail()
    {
        trail = GetComponent<TrailRenderer>();
        if (trail == null) trail = gameObject.AddComponent<TrailRenderer>();
        trail.time = 4f;
        trail.startWidth = 0.05f;
        trail.endWidth = 0f;
        trail.material = new Material(Shader.Find("Sprites/Default"));
        trail.startColor = Color.white;
        trail.endColor = new Color(1f, 1f, 1f, 0f);
    }

    void Update()
    {
        if (phase == Phase.Flying && state.Spin.sqrMagnitude > 1e-6f)
        {
            float angle = state.Spin.magnitude * spinVisualScale * Time.deltaTime * Mathf.Rad2Deg;
            VisualTarget.Rotate(state.Spin.normalized, angle, Space.World);
        }

        if (phase == Phase.Flying)
        {
            float alpha = Mathf.Clamp01((Time.time - Time.fixedTime) / Time.fixedDeltaTime);
            transform.position = Vector3.Lerp(prevPhysicsPosition, state.Position, alpha);
        }

        UpdateTrajectoryPreview();

        Keyboard kb = Keyboard.current;
        if (kb == null) return;

        if (phase == Phase.Ready)
        {
            if (kb.digit1Key.wasPressedThisFrame) hitType = HitType.Float;
            if (kb.digit2Key.wasPressedThisFrame) hitType = HitType.Topspin;
            if (kb.digit3Key.wasPressedThisFrame) hitType = HitType.Backspin;
            if (kb.digit4Key.wasPressedThisFrame) hitType = HitType.SideLeft;
            if (kb.digit5Key.wasPressedThisFrame) hitType = HitType.SideRight;
            if (kb.iKey.wasPressedThisFrame) integrator = (Integrator)(((int)integrator + 1) % 3);
            if (kb.nKey.wasPressedThisFrame) windEnabled = !windEnabled;
            if (kb.upArrowKey.wasPressedThisFrame) hitSpeed = Mathf.Min(hitSpeed + 1f, 45f);
            if (kb.downArrowKey.wasPressedThisFrame) hitSpeed = Mathf.Max(hitSpeed - 1f, 5f);
            if (kb.spaceKey.wasPressedThisFrame) HitButtonPressed();
            if (kb.bKey.wasPressedThisFrame) RunIntegratorComparison();
        }
    }

    private Vector3 GetAimDirection()
    {
        return playerCamera != null ? playerCamera.GetAimDirection() : hitDirection.normalized;
    }

    public void HitButtonPressed()
    {
        if (phase != Phase.Ready) return;

        state = BuildInitialState(GetAimDirection(), hitType, hitSpeed);
        prevPhysicsPosition = state.Position;
        flightTime = 0f;
        apexHeight = state.Position.y;
        noiseSeed = Random.Range(0f, 100f);

        flightLog.Clear();
        flightLog.AppendLine("t_s,x,y,z,vx,vy,vz,speed_ms,spin_rads");
        AddSample(0f);

        lastResult = "";
        phase = Phase.Flying;
    }

    private BallState BuildInitialState(Vector3 dir, HitType type, float speed)
    {
        dir = dir.normalized;
        Vector3 flat = new Vector3(dir.x, 0f, dir.z);
        Vector3 right = flat.sqrMagnitude > 1e-6f
            ? Vector3.Cross(Vector3.up, flat.normalized)
            : Vector3.right;

        Vector3 spin;
        switch (type)
        {
            case HitType.Topspin: spin = right * spinRate; break;
            case HitType.Backspin: spin = -right * spinRate; break;
            case HitType.SideLeft:
            case HitType.SideRight:
            {
                float tilt = sideSpinTilt * Mathf.Deg2Rad;
                float side = (type == HitType.SideRight) ? 1f : -1f;
                spin = (Vector3.up * (side * Mathf.Cos(tilt)) + right * Mathf.Sin(tilt)) * spinRate;
                break;
            }
            default: spin = right * floatSpinRate; break;
        }

        return new BallState(startPosition, dir * speed, spin);
    }

    private AirEnvironment MakeEnvironment(float t)
    {
        float tt = t + noiseSeed;
        Vector3 wind = Vector3.zero;
        if (windEnabled)
        {
            float gx = Mathf.PerlinNoise(tt * gustFrequency, 3.1f) * 2f - 1f;
            float gz = Mathf.PerlinNoise(tt * gustFrequency, 7.7f) * 2f - 1f;
            wind = baseWind + new Vector3(gx, 0f, gz) * gustStrength;
        }

        Vector2 noise = new Vector2(
            Mathf.PerlinNoise(tt * wobbleFrequency, 11.3f) * 2f - 1f,
            Mathf.PerlinNoise(tt * wobbleFrequency, 23.9f) * 2f - 1f);

        return new AirEnvironment { Wind = wind, WobbleNoise = noise };
    }

    private ForceSet ComputeForces(BallState s, AirEnvironment env)
    {
        ForceSet f = new ForceSet();
        f.Gravity = Physics.gravity * mass;

        Vector3 vRel = s.Velocity - env.Wind;
        float speed = vRel.magnitude;
        if (speed < 1e-4f) return f;
        Vector3 vHat = vRel / speed;

        float area = Mathf.PI * radius * radius;
        float q = 0.5f * airDensity * speed * speed;
        float reynolds = airDensity * speed * (2f * radius) / dynamicViscosity;

        float cd = dragCoefficient;
        if (useDragCrisis)
        {
            float t = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(
                criticalReynolds - 0.5f * criticalReynoldsWidth,
                criticalReynolds + 0.5f * criticalReynoldsWidth,
                reynolds));
            cd = Mathf.Lerp(subcriticalDragCoefficient, dragCoefficient, t);
        }
        f.Drag = -q * cd * area * vHat;

        Vector3 wCrossV = Vector3.Cross(s.Spin, vHat);
        float wPerp = wCrossV.magnitude;
        if (wPerp > 1e-6f)
        {
            float spinParameter = radius * wPerp / speed;
            float cl = Mathf.Min(liftSlope * spinParameter, maxLiftCoefficient);
            f.Magnus = q * cl * area * (wCrossV / wPerp);
        }

        if (wobbleCoefficient > 0f && (env.WobbleNoise.x != 0f || env.WobbleNoise.y != 0f))
        {
            float x = (reynolds - criticalReynolds) / criticalReynoldsWidth;
            float bump = Mathf.Exp(-x * x);
            float spinSuppression = 1f / (1f + s.Spin.magnitude / floatSpinThreshold);

            Vector3 e1 = Vector3.Cross(vHat, Vector3.up);
            if (e1.sqrMagnitude < 1e-6f) e1 = Vector3.Cross(vHat, Vector3.right);
            e1.Normalize();
            Vector3 e2 = Vector3.Cross(e1, vHat);

            f.Wobble = q * area * wobbleCoefficient * bump * spinSuppression
                       * (e1 * env.WobbleNoise.x + e2 * env.WobbleNoise.y);
        }

        return f;
    }

    private BallState Derivative(BallState s, AirEnvironment env)
    {
        ForceSet f = ComputeForces(s, env);
        return new BallState(s.Velocity, f.Total / mass, s.Spin * -spinDecayRate);
    }

    private BallState Step(Integrator method, BallState s, float dt, AirEnvironment env)
    {
        switch (method)
        {
            case Integrator.Euler:
            {
                BallState d = Derivative(s, env);
                return new BallState(
                    s.Position + d.Position * dt,
                    s.Velocity + d.Velocity * dt,
                    s.Spin + d.Spin * dt);
            }
            case Integrator.SemiImplicitEuler:
            {
                BallState d = Derivative(s, env);
                Vector3 v = s.Velocity + d.Velocity * dt;
                return new BallState(
                    s.Position + v * dt,
                    v,
                    s.Spin + d.Spin * dt);
            }
            default:
            {
                BallState k1 = Derivative(s, env);
                BallState k2 = Derivative(s + k1 * (dt * 0.5f), env);
                BallState k3 = Derivative(s + k2 * (dt * 0.5f), env);
                BallState k4 = Derivative(s + k3 * dt, env);
                return s + (k1 + k2 * 2f + k3 * 2f + k4) * (dt / 6f);
            }
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        if (phase == Phase.Landed)
        {
            landedTimer -= dt;
            if (landedTimer <= 0f) ResetBall();
            return;
        }
        if (phase != Phase.Flying) return;

        AirEnvironment env = MakeEnvironment(flightTime);
        lastForces = ComputeForces(state, env);
        BallState next = Step(integrator, state, dt, env);

        if (CrossesNet(state.Position, next.Position, out Vector3 netPoint))
        {
            EndFlight(netPoint, flightTime + dt, true);
            return;
        }

        float contactY = groundHeight + radius;
        if (next.Position.y <= contactY)
        {
            float frac = (state.Position.y - contactY) / (state.Position.y - next.Position.y);
            Vector3 landing = Vector3.Lerp(state.Position, next.Position, frac);
            EndFlight(landing, flightTime + frac * dt, false);
            return;
        }

        prevPhysicsPosition = state.Position;
        state = next;
        flightTime += dt;
        apexHeight = Mathf.Max(apexHeight, state.Position.y);

        AddSample(flightTime);
    }

    private bool CrossesNet(Vector3 a, Vector3 b, out Vector3 point)
    {
        point = b;
        if (net == null) return false;

        float netZ = net.position.z;
        float da = a.z - netZ;
        float db = b.z - netZ;
        if (da * db > 0f || Mathf.Approximately(da, db)) return false;

        float t = da / (da - db);
        point = Vector3.Lerp(a, b, t);
        bool withinWidth = Mathf.Abs(point.x - net.position.x) <= netHalfWidth;
        bool belowTop = point.y <= netHeight + radius;
        return withinWidth && belowTop;
    }

    private void EndFlight(Vector3 point, float time, bool hitNet)
    {
        state.Position = point;
        transform.position = point;
        phase = Phase.Landed;
        landedTimer = resetDelay;

        float range = new Vector2(point.x - startPosition.x, point.z - startPosition.z).magnitude;
        lastResult = (hitNet ? "Net contact" : "Landed")
            + ": range " + range.ToString("F2", CultureInfo.InvariantCulture) + " m, flight "
            + time.ToString("F2", CultureInfo.InvariantCulture) + " s, apex "
            + apexHeight.ToString("F2", CultureInfo.InvariantCulture) + " m";
        Debug.Log(lastResult + " (" + integrator + ", " + hitType + ")");

        string fileName = "flight_" + integrator + "_" + hitType + "_"
            + System.DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv";
        WriteCsv(fileName, flightLog.ToString());
    }

    private void ResetBall()
    {
        transform.position = startPosition;
        prevPhysicsPosition = startPosition;
        VisualTarget.localRotation = visualStartRotation;
        state = new BallState(startPosition, Vector3.zero, Vector3.zero);
        phase = Phase.Ready;
        if (trail != null) trail.Clear();
    }

    private static string F(float v) { return v.ToString("F4", CultureInfo.InvariantCulture); }

    private void AddSample(float t)
    {
        flightLog.AppendLine(string.Join(",", new[]
        {
            F(t),
            F(state.Position.x), F(state.Position.y), F(state.Position.z),
            F(state.Velocity.x), F(state.Velocity.y), F(state.Velocity.z),
            F(state.Velocity.magnitude), F(state.Spin.magnitude)
        }));
    }

    private void WriteCsv(string fileName, string content)
    {
        string dir = Path.Combine(Application.dataPath, "..", "TrajectoryLogs");
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, fileName);
        File.WriteAllText(path, content);
        Debug.Log("Wrote " + Path.GetFullPath(path));
    }

    private SimResult Simulate(Integrator method, BallState initial, float dt, AirEnvironment env, bool record)
    {
        SimResult r = new SimResult();
        BallState s = initial;
        float contactY = groundHeight + radius;
        float t = 0f;
        if (record) r.Positions.Add(s.Position);

        int maxSteps = Mathf.CeilToInt(30f / dt);
        for (int i = 0; i < maxSteps; i++)
        {
            BallState next = Step(method, s, dt, env);
            r.Steps++;
            if (next.Position.y <= contactY)
            {
                float frac = (s.Position.y - contactY) / (s.Position.y - next.Position.y);
                r.Landing = Vector3.Lerp(s.Position, next.Position, frac);
                r.FlightTime = t + frac * dt;
                return r;
            }
            s = next;
            t += dt;
            if (record) r.Positions.Add(s.Position);
        }
        r.Landing = s.Position;
        r.FlightTime = t;
        return r;
    }

    private float ComputeRmse(SimResult test, SimResult reference, float dt, float refDt)
    {
        int ratio = Mathf.Max(1, Mathf.RoundToInt(dt / refDt));
        int n = Mathf.Min(test.Positions.Count, (reference.Positions.Count - 1) / ratio + 1);
        if (n <= 0) return 0f;

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            float d = Vector3.Distance(test.Positions[i], reference.Positions[i * ratio]);
            sum += d * d;
        }
        return (float)System.Math.Sqrt(sum / n);
    }

    private double TimeSimulation(Integrator method, BallState initial, float dt, AirEnvironment env, int runs)
    {
        Simulate(method, initial, dt, env, false);
        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
        for (int i = 0; i < runs; i++) Simulate(method, initial, dt, env, false);
        sw.Stop();
        return sw.Elapsed.TotalMilliseconds / runs;
    }

    private void RunIntegratorComparison()
    {
        BallState initial = BuildInitialState(GetAimDirection(), hitType, hitSpeed);
        AirEnvironment env = new AirEnvironment
        {
            Wind = windEnabled ? baseWind : Vector3.zero,
            WobbleNoise = Vector2.zero
        };

        const float refDt = 0.0005f;
        SimResult reference = Simulate(Integrator.RK4, initial, refDt, env, true);

        float[] timesteps = { 0.04f, 0.02f, 0.01f, 0.005f };
        Integrator[] methods = { Integrator.Euler, Integrator.SemiImplicitEuler, Integrator.RK4 };

        StringBuilder csv = new StringBuilder();
        csv.AppendLine("# hit=" + hitType + " speed=" + F(hitSpeed) + " wind=" + windEnabled
            + " reference=RK4 dt=" + refDt + " ref_landing=("
            + F(reference.Landing.x) + ";" + F(reference.Landing.y) + ";" + F(reference.Landing.z)
            + ") ref_flight_time=" + F(reference.FlightTime));
        csv.AppendLine("integrator,dt_s,steps,force_evaluations,cpu_ms_per_run,landing_error_m,rmse_m,flight_time_error_s");

        foreach (float dt in timesteps)
        {
            foreach (Integrator m in methods)
            {
                SimResult r = Simulate(m, initial, dt, env, true);
                float rmse = ComputeRmse(r, reference, dt, refDt);
                float landingError = Vector3.Distance(r.Landing, reference.Landing);
                float flightTimeError = Mathf.Abs(r.FlightTime - reference.FlightTime);
                int evaluations = r.Steps * (m == Integrator.RK4 ? 4 : 1);
                double cpuMs = TimeSimulation(m, initial, dt, env, 200);

                csv.AppendLine(m + "," + F(dt) + "," + r.Steps + "," + evaluations + ","
                    + cpuMs.ToString("F5", CultureInfo.InvariantCulture) + ","
                    + F(landingError) + "," + F(rmse) + "," + F(flightTimeError));
            }
        }

        string fileName = "integrator_comparison_" + hitType + "_"
            + System.DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv";
        WriteCsv(fileName, csv.ToString());
        lastResult = "Comparison written to TrajectoryLogs/" + fileName;
    }

    void OnGUI()
    {
        GUI.skin.label.fontSize = 16;
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("Hit: " + hitType + "  |  " + hitSpeed.ToString("F0") + " m/s ("
            + (hitSpeed * 3.6f).ToString("F0") + " km/h)");
        sb.AppendLine("Integrator: " + integrator + "   Wind: " + (windEnabled ? "ON" : "off"));
        if (phase == Phase.Flying)
        {
            sb.AppendLine("t = " + flightTime.ToString("F2") + " s   speed = "
                + state.Velocity.magnitude.ToString("F1") + " m/s   spin = "
                + state.Spin.magnitude.ToString("F1") + " rad/s");
        }
        if (lastResult.Length > 0) sb.AppendLine(lastResult);
        sb.AppendLine();
        sb.AppendLine("[1-5] hit type  [Up/Down] speed  [I] integrator  [N] wind");
        sb.AppendLine("[Space] hit  [B] integrator comparison  [F] camera follow"
            + (playerCamera != null ? (playerCamera.followBall ? " (ON)" : " (off)") : ""));
        GUI.Label(new Rect(12, 10, 700, 220), sb.ToString());
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying || phase != Phase.Flying) return;
        Vector3 p = transform.position;
        const float scale = 0.25f;
        Gizmos.color = Color.gray;   Gizmos.DrawLine(p, p + lastForces.Gravity * scale);
        Gizmos.color = Color.red;    Gizmos.DrawLine(p, p + lastForces.Drag * scale);
        Gizmos.color = Color.green;  Gizmos.DrawLine(p, p + lastForces.Magnus * scale);
        Gizmos.color = Color.yellow; Gizmos.DrawLine(p, p + lastForces.Wobble * scale);
    }
}