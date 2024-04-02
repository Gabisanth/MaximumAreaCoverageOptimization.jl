using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FOVPlot : MonoBehaviour
{
    public GameObject circlePrefab; // Reference to the circle Prefab
    public GameObject targetDrone;
    Vector3 Drone_pos;

    public float FOV = 80f;

    void Update()
    {
        
        
        Drone_pos = targetDrone.transform.position;
        Vector3 spawnPosition = new Vector3(Drone_pos.x, 0.5f, Drone_pos.z);
        Quaternion rotation = Quaternion.Euler(90f, 0f, 0f);

        GameObject circle = Instantiate(circlePrefab, spawnPosition, rotation);
        Vector3 newScale = Vector3.one * (Mathf.Tan(FOV * (Mathf.PI/180)/2) * Drone_pos.y * 2f); // Assuming the original scale is based on a radius of 1
        circle.transform.localScale = newScale; // Set the new scale
    }

}
