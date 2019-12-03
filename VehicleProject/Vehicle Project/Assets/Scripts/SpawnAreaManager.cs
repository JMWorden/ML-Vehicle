using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnAreaManager : MonoBehaviour
{
    public Bounds[] SpawnAreas;

    // Start is called before the first frame update
    void Start()
    {
        BoxCollider[] colliders = GetComponentsInChildren<BoxCollider>();
        List<Bounds> SpawnAreaList = new List<Bounds>();

        foreach (BoxCollider collider in colliders) {
            SpawnAreaList.Add(collider.bounds);
            collider.enabled = false;
        }

        SpawnAreas = SpawnAreaList.ToArray();

        Debug.Log("SpawnAreaManager found " + SpawnAreas.Length + " spawn areas");
    }

    public Vector3 getSpawnPoint() {
        Bounds bounds = SpawnAreas[Random.Range(0, SpawnAreas.Length)];
        return new Vector3(Random.Range(bounds.min.x, bounds.max.x), 0f, 
            Random.Range(bounds.min.z, bounds.max.z));
    }
}