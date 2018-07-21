using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CarEngine : MonoBehaviour {

    public float force = 1;
	public Transform path;
	public float maxSteerAngle = 40f;
	public WheelCollider wheelFL;
	public WheelCollider wheelFR;

    private Rigidbody rb;
	private List<Transform> nodes;
	private int currentNode = 0;

	private void Start () {
        rb = GetComponent<Rigidbody>();

		// Get child transforms
		Transform[] pathTransforms = path.GetComponentsInChildren<Transform> ();
		nodes = new List<Transform> ();

		// create list skipping the parent transform
		for (int i = 0; i < pathTransforms.Length; i++) {
			if (pathTransforms [i] != path.transform) {
				nodes.Add (pathTransforms [i]);
			}
		}
	}
	
	private void Update () {
        if (Input.GetKey(KeyCode.W)) {
            rb.AddForce(transform.forward * force);
        }
        if (Input.GetKey(KeyCode.A)) {
            rb.AddTorque(transform.up * -force);
        }
	}

	private void FixedUpdate() {
		ApplySteer ();
	}

	private void ApplySteer() {
		Vector3 relativeVector = transform.InverseTransformPoint (nodes [currentNode].position);
		float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
		wheelFL.steerAngle = newSteer;
		wheelFR.steerAngle = newSteer;
		relativeVector = relativeVector / relativeVector.magnitude;

	}
}
