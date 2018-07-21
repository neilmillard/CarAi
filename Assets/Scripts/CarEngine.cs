using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CarEngine : MonoBehaviour {

    public float force = 1;
	public Transform path;
	public float maxSteerAngle = 40f;
	public WheelCollider wheelFL;
	public WheelCollider wheelFR;
	public WheelCollider wheelRL;
	public WheelCollider wheelRR;
	public float maxMotorTorque = 50f;
	public float maxBrakeTorque = 150f;
	public float currentSpeed;
	public float maximumSpeed = 100f;
	public Vector3 centerOfMass;
	public bool isBraking = false;
	public Texture2D textureNormal;
	public Texture2D textureBraking;
	public Renderer carRenderer;

	[Header("Sensors")]
	public float sensorLength = 3f;
	public Vector3 frontSensorPosition = new Vector3(0f, 0.2f, 0.5f);
	public float sideSensorPosition = 0.3f;
	public float frontSensorAngle = 30f;


    private Rigidbody rb;
	private List<Transform> nodes;
	private int currentNode = 0;
	private float currentTorque = 0;

	private void Start () {
        rb = GetComponent<Rigidbody>();
		rb.centerOfMass = centerOfMass;
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
        
	}

	private void FixedUpdate() {
		Sensors ();
		ApplySteer ();
		Drive();
		CheckWaypointDistance ();
		Braking ();
	}

	private void Sensors ()
	{
		RaycastHit hit;
		Vector3 sensorStartPos = transform.position;
		sensorStartPos += transform.forward * frontSensorPosition.z;
		sensorStartPos += transform.up * frontSensorPosition.y;

		// front center sensor

		if (Physics.Raycast (sensorStartPos, transform.forward, out hit, sensorLength)) {
			Debug.DrawLine (sensorStartPos, hit.point);
		}

		// front right sensor
		sensorStartPos.x += sideSensorPosition;
		if (Physics.Raycast (sensorStartPos, transform.forward, out hit, sensorLength)) {
			Debug.DrawLine (sensorStartPos, hit.point);
		}

		// front right angle sensor
		if (Physics.Raycast (sensorStartPos, Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength)) {
			Debug.DrawLine (sensorStartPos, hit.point);
		}

		// front left sensor
		sensorStartPos.x -= 2f * sideSensorPosition;
		if (Physics.Raycast (sensorStartPos, transform.forward, out hit, sensorLength)) {
			Debug.DrawLine (sensorStartPos, hit.point);
		}
		// front right angle sensor
		if (Physics.Raycast (sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward, out hit, sensorLength)) {
			Debug.DrawLine (sensorStartPos, hit.point);
		}

	}

	private void ApplySteer() {
		Vector3 relativeVector = transform.InverseTransformPoint (nodes [currentNode].position);
		float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
		wheelFL.steerAngle = newSteer;
		wheelFR.steerAngle = newSteer;
		relativeVector = relativeVector / relativeVector.magnitude;

	}

	private void Drive(){
		currentSpeed = 2 * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60 / 1000;
		if (currentSpeed < maximumSpeed && !isBraking) {
			currentTorque = maxMotorTorque;
		} else {
			currentTorque = 0f;
		}
		wheelFL.motorTorque = currentTorque;
		wheelFR.motorTorque = currentTorque;
	}

	private void CheckWaypointDistance(){
		float distanceToNode = Vector3.Distance (transform.position, nodes [currentNode].position);
		if ( distanceToNode < 0.5f) {
			if (currentNode == nodes.Count - 1) {
				currentNode = 0;
			} else {
				currentNode++;
			}
		}
	}

	private void Braking(){
		if (isBraking) {
			carRenderer.material.mainTexture = textureBraking;
			wheelRL.brakeTorque = maxBrakeTorque;
			wheelRR.brakeTorque = maxBrakeTorque;
		} else {
			carRenderer.material.mainTexture = textureNormal;
			wheelRL.brakeTorque = 0f;
			wheelRR.brakeTorque = 0f;
		}
	}

}
