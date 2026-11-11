using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerCamera : MonoBehaviour
{
    [Header("What to orbit around")]
    public Transform target;

    [Header("Aiming")]
    public float rotationSpeed = 0.2f;
    public float minPitch = -89f;
    public float maxPitch = 89f;

    [Header("Camera position")]
    public float distance = 2f;
    public float minDistance = 2f;
    public float maxDistance = 12f;
    [Tooltip("Lowest angle (degrees above the ball's height) the camera may sit at. Higher = a more top-down view, so the preview arc and landing marker are easier to see.")]
    [Range(0f, 80f)]
    public float minCameraPitch = 35f;
    public float groundHeight = 0f;
    [Tooltip("The camera is never allowed lower than ground height + this.")]
    public float minCameraHeight = 0.5f;

    [Header("Follow")]
    public bool followBall = false;
    public float followSmoothing = 0.1f;

    private float yaw = 0f;
    private float pitch = 20f;
    private Vector3 pivot;
    private Vector3 homePosition;

    void Start()
    {
        if (target != null)
        {
            homePosition = target.position;
            pivot = homePosition;
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        Keyboard keyboard = Keyboard.current;
        if (keyboard != null && keyboard.fKey.wasPressedThisFrame)
        {
            followBall = !followBall;
        }

        Mouse mouse = Mouse.current;
        if (mouse != null)
        {
            if (mouse.leftButton.isPressed)
            {
                Vector2 delta = mouse.delta.ReadValue();
                yaw += delta.x * rotationSpeed;
                pitch -= delta.y * rotationSpeed;
                pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
            }

            float scroll = mouse.scroll.ReadValue().y;
            distance -= scroll * 0.005f;
            distance = Mathf.Clamp(distance, minDistance, maxDistance);
        }

        Vector3 desiredPivot = followBall ? target.position : homePosition;
        if (followSmoothing > 0f)
        {
            float blend = 1f - Mathf.Exp(-Time.deltaTime / followSmoothing);
            pivot = Vector3.Lerp(pivot, desiredPivot, blend);
        }
        else
        {
            pivot = desiredPivot;
        }

        float cameraPitch = Mathf.Clamp(pitch, minCameraPitch, maxPitch);
        Quaternion cameraRotation = Quaternion.Euler(cameraPitch, yaw, 0f);
        Vector3 position = pivot + cameraRotation * new Vector3(0f, 0f, -distance);
        position.y = Mathf.Max(position.y, groundHeight + minCameraHeight);
        transform.position = position;
        transform.LookAt(pivot);
    }

    public Vector3 GetAimDirection()
    {
        return Quaternion.Euler(pitch, yaw, 0f) * Vector3.forward;
    }
}