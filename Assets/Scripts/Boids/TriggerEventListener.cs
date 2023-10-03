using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Events;

[RequireComponent(typeof(SphereCollider))]
public class TriggerEventListener : MonoBehaviour
{
    private Boid owner;
    private SphereCollider trigger;

    public UnityAction<Boid> onEnter;
    public UnityAction<Boid> onExit;

    public float radius { set { if (trigger != null) trigger.radius = value; } }

    void Awake()
    {
        owner = GetComponentInParent<Boid>();
        trigger = GetComponent<SphereCollider>();
        trigger.isTrigger = true;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.isTrigger)
        {
            return;
        }

        Boid neighbor = other.GetComponentInParent<Boid>();
        if (neighbor == null || neighbor == owner)
        {
            return;
        }

        onEnter?.Invoke(neighbor);
    }

    void OnTriggerExit(Collider other)
    {
        if (other.isTrigger)
        {
            return;
        }

        Boid neighbor = other.GetComponentInParent<Boid>();
        if (neighbor == null || neighbor == owner)
        {
            return;
        }

        onExit?.Invoke(neighbor);
    }
}
