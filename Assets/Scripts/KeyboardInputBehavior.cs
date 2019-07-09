using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardInputBehavior : MonoBehaviour
{

    private bool anyKeyDown = false;
    public bool initialized;

    // Update is called once per frame
	public void Initialize(){
		initialized = true;
	}
    void Update()
    {
        if (initialized)
        {

            if (Input.GetKeyDown(KeyCode.UpArrow))
            {

                SendMessageBehavior.Instance.SendPacket("l");

            }
            else if (Input.GetKeyDown(KeyCode.DownArrow))
            {

                SendMessageBehavior.Instance.SendPacket("r");

            }
            else if (Input.GetKeyDown(KeyCode.LeftArrow))
            {

                SendMessageBehavior.Instance.SendPacket("f");

            }
            else if (Input.GetKeyDown(KeyCode.RightArrow))
            {

                SendMessageBehavior.Instance.SendPacket("b");

            }
            else if (Input.GetKeyDown(KeyCode.Space))
            {
                SendMessageBehavior.Instance.SendPacket("a");
            }

            if (Input.anyKey)
            {

                anyKeyDown = true;

            }
            else if (!Input.anyKey && anyKeyDown)
            {

                anyKeyDown = false;
                SendMessageBehavior.Instance.SendPacket("s");

            }
        }
    }
}

