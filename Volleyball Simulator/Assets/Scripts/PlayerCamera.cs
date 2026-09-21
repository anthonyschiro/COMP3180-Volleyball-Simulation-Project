using UnityEngine;
using UnityEngine.InputSystem; // requires the Input System package


public class PlayerCamera : MonoBehaviour
{
    [Header("What to orbit around")]
    public Transform target;

    [Header("Orbit settings")]
    public float distance = 2f;
    public float minDistance = 2f;
    public float maxDistance = 12f;
    public float rotationSpeed = 150f;
    public float minPitch = -89f;
    public float maxPitch = 89f;
    private float yaw = 0f;
    private float pitch = 20f;
    private Vector3 pivot;

    void Start()
    {
        if (target != null) pivot = target.position;
    }

    void LateUpdate()
    {
        if (target == null) return;

        Mouse mouse = Mouse.current;
        if (mouse == null) return;

        if (mouse.leftButton.isPressed)
        {
            Vector2 delta = mouse.delta.ReadValue();
            yaw += delta.x * rotationSpeed * 0.01f * Time.deltaTime;
            pitch -= delta.y * rotationSpeed * 0.01f * Time.deltaTime;
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        }

        float scroll = mouse.scroll.ReadValue().y;
        distance -= scroll * 0.005f;
        distance = Mathf.Clamp(distance, minDistance, maxDistance);

        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 offset = rotation * new Vector3(0f, 0f, -distance);
        transform.position = pivot + offset;
        transform.LookAt(pivot);
    }

    public Vector3 GetAimDirection()
    {
        return transform.forward;
    }
}