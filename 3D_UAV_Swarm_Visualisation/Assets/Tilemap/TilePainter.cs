using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class TilePainter : MonoBehaviour
{

    public Tile redSquare;

    public Tilemap tilemap;

    public TextAsset textAssetData;

    //[ContextMenu("Paint")]

    // void Paint(){

    //     for (int i = 0; i <= 1; i++)
    //     {
    //         Vector3Int myVector = new Vector3Int(position[0] + i, position[1] +i, position[2] +i);
    //         tilemap.SetTile(myVector, redSquare);
    //     }
        
        
    //     //tilemap.SetTile(position, redSquare);

    // }

    private float delayTimer = 2f; // Set the delay time to 3 seconds
    private float timer; // Timer to keep track of time

    int row_number = 0;

    void Update(){

        timer += Time.deltaTime; // Increment timer based on elapsed time

        if (timer >= delayTimer) // Check if the delay time has passed
        {
            string[] lines = textAssetData.text.Split('\n');
            

            string[] cells = lines[row_number].Split(',');

            string[] nonEmptycells = FilterEmptyFields(cells);
            Debug.Log(row_number);

            for (int i = 0; i < nonEmptycells.Length; i += 5)
            {

                Vector3Int myVector = new Vector3Int( Mathf.RoundToInt(float.Parse(nonEmptycells[i+1])), Mathf.RoundToInt(float.Parse(nonEmptycells[i])), -1);
                tilemap.SetTile(myVector, redSquare);
            }
            
            timer = 0f; // Reset the timer

            row_number = row_number + 1;
        }


    }

    string[] FilterEmptyFields(string[] fields)
    {
        // Create a list to store non-empty fields
        var nonEmptyFieldsList = new System.Collections.Generic.List<string>();

        // Iterate through the fields and add non-empty ones to the list
        foreach (string field in fields)
        {
            if (!string.IsNullOrEmpty(field.Trim()))
            {
                nonEmptyFieldsList.Add(field);
            }
        }

        // Convert the list back to an array
        return nonEmptyFieldsList.ToArray();
    }

}
