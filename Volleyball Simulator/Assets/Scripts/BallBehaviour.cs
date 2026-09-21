using UnityEngine;
using UnityEngine.UI;

public class BallBehaviour : MonoBehaviour
{
    [Header("Aiming (leave empty to fall back to Hit Direction below)")]
    public PlayerCamera playerCamera;

    [Header("Launch settings")]
    public Vector3 hitDirection = new Vector3(0f, 1f, 1f);
    public float hitForce = 30f;

    [Header("Reset position (where the ball returns to after landing)")]
    public Vector3 startPosition = new Vector3(0f, 3f, 6f);

    [Header("Ball properties (for drag)")]
    public float mass = 0.27f;
    public float radius = 0.105f;
    public float dragCoefficient = 0.17f; // Constant based on a science article
    public float airDensity = 1.2f;

    [Header("Current flight state")]
    public Vector3 velocity;
    public bool isFlying = false;

    void Start()
    {
        transform.position = startPosition;
        transform.localScale = Vector3.one * (radius * 2f);
    }

    private struct BallState
    {
        public Vector3 Position;
        public Vector3 Velocity;

        public BallState(Vector3 position, Vector3 velocity)
        {
            Position = position;
            Velocity = velocity;
        }

        public static BallState operator +(BallState a, BallState b) =>
            new BallState(a.Position + b.Position, a.Velocity + b.Velocity);

        public static BallState operator *(BallState s, float scalar) =>
            new BallState(s.Position * scalar, s.Velocity * scalar);
    }

    private BallState ComputeDerivative(BallState state)
    {
        Vector3 velocityNow = state.Velocity;
        float speed = velocityNow.magnitude;

        float crossSectionalArea = Mathf.PI * radius * radius;
        Vector3 dragForce = speed > 0.0001f
            ? -0.5f * airDensity * dragCoefficient * crossSectionalArea * speed * velocityNow
            : Vector3.zero;

        Vector3 dragAcceleration = dragForce / mass;
        Vector3 acceleration = Physics.gravity + dragAcceleration;

        return new BallState(velocityNow, acceleration);
    }

    private BallState RK4Step(BallState state, float dt)
    {
        BallState k1 = ComputeDerivative(state);
        BallState k2 = ComputeDerivative(state + k1 * (dt * 0.5f));
        BallState k3 = ComputeDerivative(state + k2 * (dt * 0.5f));
        BallState k4 = ComputeDerivative(state + k3 * dt);

        BallState weightedSum = k1 + k2 * 2f + k3 * 2f + k4;
        return state + weightedSum * (dt / 6f);
    }

    void FixedUpdate()
    {
        if (!isFlying) return;

        BallState current = new BallState(transform.position, velocity);
        BallState next = RK4Step(current, Time.fixedDeltaTime);

        transform.position = next.Position;
        velocity = next.Velocity;

        if (transform.position.y <= 0f)
        {
            isFlying = false;
            velocity = Vector3.zero;
            transform.position = startPosition;
        }
    }

    public void HitButtonPressed()
    {
        Vector3 direction = playerCamera != null
            ? playerCamera.GetAimDirection()
            : hitDirection.normalized;

        velocity = direction * hitForce;
        isFlying = true;
    }
}