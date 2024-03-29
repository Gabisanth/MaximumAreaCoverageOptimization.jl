using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class DroneMovement : MonoBehaviour
{
    public TextAsset textAssetData;

    [System.Serializable]

    public class Drone{
        public float[] x;
        public float[] y;
        public float[] z;
        public float[] a;
        public float[] b;
        public float[] c;
        public float[] d;
    }
    


    public Drone Drone1 = new Drone();

    public int tableSize;
    void Start()
    {
        string[] data = textAssetData.text.Split(new string[] {",", "\n"}, StringSplitOptions.None);

        tableSize = data.Length / 7 - 1; //Don't want to include the column titles.


        Drone1.x = new float[tableSize];
        Drone1.y = new float[tableSize];
        Drone1.z = new float[tableSize];
        Drone1.a = new float[tableSize];
        Drone1.b = new float[tableSize];
        Drone1.c = new float[tableSize];
        Drone1.d = new float[tableSize];


        for (int i=0; i < tableSize; i++)
        {
            Drone1.x[i] = float.Parse(data[7 * (i + 1)]);
            Drone1.y[i] = float.Parse(data[7 * (i + 1) + 1]);
            Drone1.z[i] = float.Parse(data[7 * (i + 1) + 2]);

            Drone1.a[i] = float.Parse(data[7 * (i + 1) + 3]);
            Drone1.b[i] = float.Parse(data[7 * (i + 1) + 4]);
            Drone1.c[i] = float.Parse(data[7 * (i + 1) + 5]);
            Drone1.c[i] = float.Parse(data[7 * (i + 1) + 6]);
        }

        //StartCoroutine( Plot() );
        
    }
    




    int i = 0;
    int increment = 1;
    private float delayTimer = 0.1f; // Set the delay time to 3 seconds
    private float timer; // Timer to keep track of time

    void Update(){
        // StartCoroutine( Plot() );

        timer += Time.deltaTime; // Increment timer based on elapsed time

    if (timer >= delayTimer) // Check if the delay time has passed
        {
            transform.position = new Vector3(Drone1.y[i], Drone1.z[i], Drone1.x[i]);
            Quaternion rotationQuaternion = new Quaternion(Drone1.a[i], Drone1.b[i], Drone1.c[i], Drone1.d[i]);

            transform.rotation = rotationQuaternion;

            if (i < tableSize-1)
            {
                i = i + increment;
            }
            
            timer = 0f; // Reset the timer
        }


    }
        
        

    

}
