/*
Copyright 2015 Pim de Witte
All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;

/// <summary>
/// A thread-safe class which holds a queue with actions to execute on the next Update() method. It can be used to make calls to the main thread for
/// things such as UI Manipulation in Unity.
/// </summary>
public class UnityMainThreadDispatcher : MonoBehaviour
{
    private static readonly Queue<Action> _executionQueue = new Queue<Action>();

    private static UnityMainThreadDispatcher _instance = null;

    /// <summary>
    /// Singleton instance accessor.
    /// </summary>
    public static UnityMainThreadDispatcher Instance()
    {
        if (_instance == null)
        {
            // Versuch, eine existierende Instanz zu finden
            _instance = FindObjectOfType<UnityMainThreadDispatcher>();
            if (_instance == null)
            {
                // Erstelle eine neue Instanz, falls keine vorhanden ist
                GameObject dispatcher = new GameObject("UnityMainThreadDispatcher");
                _instance = dispatcher.AddComponent<UnityMainThreadDispatcher>();
                DontDestroyOnLoad(dispatcher);
                Debug.Log("UnityMainThreadDispatcher wurde erstellt.");
            }
        }
        return _instance;
    }

    void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(this.gameObject);
            Debug.Log("UnityMainThreadDispatcher wurde initialisiert.");
        }
        else if (_instance != this)
        {
            Destroy(this.gameObject);
            Debug.Log("Doppelte UnityMainThreadDispatcher-Instanz zerstört.");
        }
    }

    void Update()
    {
        lock (_executionQueue)
        {
            while (_executionQueue.Count > 0)
            {
                _executionQueue.Dequeue().Invoke();
            }
        }
    }

    /// <summary>
    /// Locks the queue and adds the IEnumerator to the queue
    /// </summary>
    /// <param name="action">IEnumerator function that will be executed from the main thread.</param>
    public void Enqueue(IEnumerator action)
    {
        lock (_executionQueue)
        {
            _executionQueue.Enqueue(() =>
            {
                StartCoroutine(action);
            });
        }
    }

    /// <summary>
    /// Locks the queue and adds the Action to the queue
    /// </summary>
    /// <param name="action">function that will be executed from the main thread.</param>
    public void Enqueue(Action action)
    {
        Enqueue(ActionWrapper(action));
    }

    /// <summary>
    /// Locks the queue and adds the Action to the queue, returning a Task which is completed when the action completes
    /// </summary>
    /// <param name="action">function that will be executed from the main thread.</param>
    /// <returns>A Task that can be awaited until the action completes</returns>
    public Task EnqueueAsync(Action action)
    {
        var tcs = new TaskCompletionSource<bool>();

        void WrappedAction()
        {
            try
            {
                action();
                tcs.TrySetResult(true);
            }
            catch (Exception ex)
            {
                tcs.TrySetException(ex);
            }
        }

        Enqueue(ActionWrapper(WrappedAction));
        return tcs.Task;
    }

    IEnumerator ActionWrapper(Action a)
    {
        a();
        yield return null;
    }

    /// <summary>
    /// Checks if an instance exists.
    /// </summary>
    /// <returns>True if an instance exists, false otherwise.</returns>
    public static bool Exists()
    {
        return _instance != null;
    }
}
