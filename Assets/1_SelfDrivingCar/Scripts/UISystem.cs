using UnityEngine;
using UnityEngine.UI;
using UnityStandardAssets.Vehicles.Car;
using UnityEngine.SceneManagement;
using System;
using System.Linq;

public class UISystem : MonoSingleton<UISystem>
{
    public CarManager carManager;
    public string GoodCarStatusMessage;
    public string BadSCartatusMessage;
    public Text MPH_Text;
    public Image MPH_Animation;
    public Text Angle_Text;
    public Text RecordStatus_Text;
    public Text SaveStatus_Text;
    public Text Text_CarId;
    public Text SectorNumber_Text;
    public Text Text_Loss_Value;
    public Text Text_Unc_Value;
    public Text Text_DriveMode;
    public Text OOT;
    public GameObject RecordingPause;
    public GameObject RecordDisabled;
    public bool isTraining = false;

    private bool recording;
    private float topSpeed;
    private bool saveRecording;
    private CarController ActiveCarController;

    /// <summary>
    /// Initializes the UI system and sets initial values.
    /// </summary>
    void Start()
    {
        ActiveCarController = GetComponent<CarController>();
        if (ActiveCarController != null)
        {
            topSpeed = ActiveCarController.MaxSpeed;
        }
        else
        {
            topSpeed = 200f;
        }
        recording = false;
        RecordingPause.SetActive(false);
        RecordStatus_Text.text = "RECORD";
        SaveStatus_Text.text = "";
        if (carManager != null && carManager.ActiveCarInfo != null)
        {
            SetAngleValue(carManager.ActiveCarInfo.currentTelemetry.steering_angle);
            SetMPHValue(carManager.ActiveCarInfo.currentTelemetry.throttle);
            SetLapNumber(carManager.ActiveCarInfo.CarId);
        }
        if (!isTraining)
        {
            RecordDisabled.SetActive(true);
            RecordStatus_Text.text = "";
            SetConfidenceColor(Color.green);
        }
    }

    /// <summary>
    /// Updates the loss value text.
    /// </summary>
    public void SetLossValue(float value)
    {
        Text_Loss_Value.text = value.ToString("#000.00");
    }

    /// <summary>
    /// Updates the out-of-track value text.
    /// </summary>
    public void SetOutOfTrackValue(int value)
    {
        OOT.text = value.ToString("#0");
    }

    /// <summary>
    /// Updates the uncertainty value text.
    /// </summary>
    public void SetUncertaintyValue(float value)
    {
        Text_Unc_Value.text = value.ToString("#0.0000000000");
    }

    /// <summary>
    /// Updates the lap number text.
    /// </summary>
    public void SetLapNumber(int value)
    {
        Text_CarId.text = value.ToString();
    }

    /// <summary>
    /// Updates the sector number text.
    /// </summary>
    public void SetSectorNumber(int value, int totalSectors)
    {
        SectorNumber_Text.text = value.ToString() + " / " + totalSectors.ToString();
    }

    /// <summary>
    /// Updates the confidence color of the loss value text.
    /// </summary>
    public void SetConfidenceColor(Color color)
    {
        Text_Loss_Value.color = color;
    }

    /// <summary>
    /// Updates the steering angle text.
    /// </summary>
    public void SetAngleValue(float value)
    {
        Angle_Text.text = value.ToString("N2") + "°";
    }

    /// <summary>
    /// Updates the MPH value text and its animation fill.
    /// </summary>
    public void SetMPHValue(float value)
    {
        MPH_Text.text = value.ToString("N2");
        MPH_Animation.fillAmount = value / topSpeed;
    }

    /// <summary>
    /// Sets the drive mode text based on whether the car is autonomous.
    /// </summary>
    public void SetMode(bool autonomous)
    {
        Text_DriveMode.text = autonomous ? "Agent" : "NPC";
    }

    /// <summary>
    /// Toggles the recording state and updates the UI accordingly.
    /// </summary>
    public void ToggleRecording()
    {
        if (!isTraining)
            return;
        if (!recording)
        {
            if (ActiveCarController != null && ActiveCarController.checkSaveLocation())
            {
                recording = true;
                RecordingPause.SetActive(true);
                RecordStatus_Text.text = "RECORDING";
                ActiveCarController.IsRecording = true;
            }
        }
        else
        {
            saveRecording = true;
            if (ActiveCarController != null)
                ActiveCarController.IsRecording = false;
        }
    }

    /// <summary>
    /// Updates car-related values on the UI.
    /// </summary>
    void UpdateCarValues()
    {
        if (carManager != null && carManager.ActiveCarInfo != null)
        {
            SetAngleValue(carManager.ActiveCarInfo.currentTelemetry.steering_angle);
            SetMPHValue(carManager.ActiveCarInfo.currentTelemetry.speed);
            SetLapNumber(carManager.ActiveCarInfo.CarId);
            SetMode(carManager.ActiveCarInfo.autonomous);
        }
        else
        {
            carManager = FindObjectsOfType<CarManager>().First();
            SetMPHValue(0);
            SetAngleValue(0);
        }
    }

    /// <summary>
    /// Updates the UI every frame, handling recording status, input, and car values.
    /// </summary>
    void Update()
    {
        if (ActiveCarController != null && ActiveCarController.getSaveStatus())
        {
            SaveStatus_Text.text = "Capturing Data: " + (int)(100 * ActiveCarController.getSavePercent()) + "%";
        }
        else if (saveRecording)
        {
            SaveStatus_Text.text = "";
            recording = false;
            RecordingPause.SetActive(false);
            RecordStatus_Text.text = "RECORD";
            saveRecording = false;
        }
        if (Input.GetKeyDown(KeyCode.R))
            ToggleRecording();
        if (Input.GetKeyDown(KeyCode.Escape))
            SceneManager.LoadScene("MenuScene");
        UpdateCarValues();
    }
}
