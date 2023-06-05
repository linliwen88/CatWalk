using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class CatAgent : Agent
{
    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    [Tooltip(
        "The speed the agent will try to match.\n\n" +
        "TRAINING:\n" +
        "For VariableSpeed envs, this value will randomize at the start of each training episode.\n" +
        "Otherwise the agent will try to match the speed set here.\n\n" +
        "INFERENCE:\n" +
        "During inference, VariableSpeed agents will modify their behavior based on this value " +
        "whereas the CrawlerDynamic & CrawlerStatic agents will run at the speed specified during training "
    )]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15; //The max walking speed

    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")] [Space(10)] 
    public Transform root; // for episode begin random rotation
    public Transform pelvis;
    public Transform abdomen;
    public Transform chest;
    public Transform neck;
    public Transform flLeg_1;
    public Transform flLeg_2;
    public Transform flLeg_3;
    public Transform flLeg_4;
    public Transform frLeg_1;
    public Transform frLeg_2;
    public Transform frLeg_3;
    public Transform frLeg_4;
    public Transform blLeg_1;
    public Transform blLeg_2;
    public Transform blLeg_3;
    public Transform blLeg_4;
    public Transform brLeg_1;
    public Transform brLeg_2;
    public Transform brLeg_3;
    public Transform brLeg_4;
    public Transform tail;
    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    [Space(10)]
    public bool useFootGroundedVisualization;

    public MeshRenderer foot0;
    public MeshRenderer foot1;
    public MeshRenderer foot2;
    public MeshRenderer foot3;
    public Material groundedMaterial;
    public Material unGroundedMaterial;

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position); //spawn target

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        //Setup each body part
        m_JdController.SetupBodyPart(pelvis);
        m_JdController.SetupBodyPart(abdomen);
        m_JdController.SetupBodyPart(chest);
        m_JdController.SetupBodyPart(neck);
        m_JdController.SetupBodyPart(flLeg_1);
        m_JdController.SetupBodyPart(flLeg_2);
        m_JdController.SetupBodyPart(flLeg_3);
        m_JdController.SetupBodyPart(flLeg_4);
        m_JdController.SetupBodyPart(frLeg_1);
        m_JdController.SetupBodyPart(frLeg_2);
        m_JdController.SetupBodyPart(frLeg_3);
        m_JdController.SetupBodyPart(frLeg_4);
        m_JdController.SetupBodyPart(blLeg_1);
        m_JdController.SetupBodyPart(blLeg_2);
        m_JdController.SetupBodyPart(blLeg_3);
        m_JdController.SetupBodyPart(blLeg_4);
        m_JdController.SetupBodyPart(brLeg_1);
        m_JdController.SetupBodyPart(brLeg_2);
        m_JdController.SetupBodyPart(brLeg_3);
        m_JdController.SetupBodyPart(brLeg_4);
        m_JdController.SetupBodyPart(tail);

    }

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        root.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        //Set our goal walking speed
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != pelvis)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        //velocity we want to match
        var velGoal = cubeForward * TargetWalkingSpeed;
        //ragdoll's avg vel
        var avgVel = GetAvgVelocity();

        //current ragdoll velocity. normalized
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //avg body vel relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        //vel goal relative to cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        //rotation delta
        sensor.AddObservation(Quaternion.FromToRotation(pelvis.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(pelvis.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
            sensor.AddObservation(1);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        // Pick a new target joint rotation
        bpDict[abdomen].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[chest].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[neck].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[flLeg_1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[flLeg_2].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[flLeg_3].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[flLeg_4].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[frLeg_1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[frLeg_2].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[frLeg_3].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[frLeg_4].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[blLeg_1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[blLeg_2].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[blLeg_3].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[blLeg_4].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[brLeg_1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[brLeg_2].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[brLeg_3].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[brLeg_4].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        bpDict[tail].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        // Update joint strength
        bpDict[abdomen].SetJointStrength(continuousActions[++i]);
        bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[neck].SetJointStrength(continuousActions[++i]);
        bpDict[flLeg_1].SetJointStrength(continuousActions[++i]);
        bpDict[flLeg_2].SetJointStrength(continuousActions[++i]);
        bpDict[flLeg_3].SetJointStrength(continuousActions[++i]);
        bpDict[flLeg_4].SetJointStrength(continuousActions[++i]);
        bpDict[frLeg_1].SetJointStrength(continuousActions[++i]);
        bpDict[frLeg_2].SetJointStrength(continuousActions[++i]);
        bpDict[frLeg_3].SetJointStrength(continuousActions[++i]);
        bpDict[frLeg_4].SetJointStrength(continuousActions[++i]);
        bpDict[blLeg_1].SetJointStrength(continuousActions[++i]);
        bpDict[blLeg_2].SetJointStrength(continuousActions[++i]);
        bpDict[blLeg_3].SetJointStrength(continuousActions[++i]);
        bpDict[blLeg_4].SetJointStrength(continuousActions[++i]);
        bpDict[brLeg_1].SetJointStrength(continuousActions[++i]);
        bpDict[brLeg_2].SetJointStrength(continuousActions[++i]);
        bpDict[brLeg_3].SetJointStrength(continuousActions[++i]);
        bpDict[brLeg_4].SetJointStrength(continuousActions[++i]);
        bpDict[tail].SetJointStrength(continuousActions[++i]);
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        // If enabled the feet will light up green when the foot is grounded.
        // This is just a visualization and isn't necessary for function
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[flLeg_4].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[frLeg_4].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[blLeg_4].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[brLeg_4].groundContact.touchingGround
                ? groundedMaterial
                : unGroundedMaterial;
        }

        var cubeForward = m_OrientationCube.transform.forward;

        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * TargetWalkingSpeed, GetAvgVelocity());

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var lookAtTargetReward = (Vector3.Dot(cubeForward, pelvis.forward) + 1) * .5F;

        AddReward(matchSpeedReward * lookAtTargetReward);
    }

    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(pelvis, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    /// <summary>
    ///Returns the average velocity of all of the body parts
    ///Using the velocity of the body only has shown to result in more erratic movement from the limbs
    ///Using the average helps prevent this erratic movement
    /// </summary>
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        Vector3 avgVel = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        avgVel = velSum / numOfRb;
        return avgVel;
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }
}
