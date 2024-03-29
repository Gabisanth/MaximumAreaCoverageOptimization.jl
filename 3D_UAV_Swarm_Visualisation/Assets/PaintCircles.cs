using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PaintCircles : MonoBehaviour
{
    public Shader exploredAreaShader; // Reference to your custom shader

    // Material to apply the shader
    private Material material;

    
    
    float i = 0.0f;

    public GameObject targetDrone;
    Vector3 Drone_pos;

    void Update()
    {
        // Get the material attached to the renderer
        Renderer renderer = GetComponent<Renderer>();
        material = renderer.material;

        // Ensure the shader on the material is set to the custom shader
        material.shader = exploredAreaShader;
        // Assuming you want to draw a circle at position (1, 0, 1) with radius 2 and red color

        Drone_pos = targetDrone.transform.position;

        DrawCircle(new Vector3(Drone_pos.x, 1, Drone_pos.z), 20f, Color.black);
        i = i + 1f;
    }
    

    // Method to draw a circle at a specific center position and radius
    public void DrawCircle(Vector3 center, float radius, Color areaColor)
    {
        // Pass center position and radius to the shader
        material.SetVector("_Center", center);
        material.SetFloat("_Radius", radius);
        material.SetColor("_AreaColor", areaColor);
    }
}
