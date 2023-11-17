using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;

public class WorldLocalisationAdjustment : MonoBehaviour
{
    public Transform interactor;    // object used to manipulate the target
    public Transform target;        // the transform to be adjusted

    public Interactable rotateButton;
    public Interactable translateButton;
    public Interactable noneButton;
    public Interactable resetButton;
    public Interactable makeFlatButton;
    public Interactable scanAgainButton;

    public GameObject rotateInteractor;
    public GameObject translateInteractor;
    public ImageTrackingBehaviour imageTrackingBehaviour;

    public LineRenderer lineRenderer;

    private bool rotating = false;
    private bool translating = false;

    enum Manip {None, Rotate, Translate};
    Manip manipulationType;

    private Quaternion initialRotationOffset;
    private Vector3 initialTranslationOffset;

    // Start is called before the first frame update
    void Start()
    {
        manipulationType = Manip.None;
        rotateButton.OnClick.AddListener(   () => SelectManipulationType(Manip.Rotate));
        translateButton.OnClick.AddListener(() => SelectManipulationType(Manip.Translate ));
        noneButton.OnClick.AddListener(     () => SelectManipulationType(Manip.None));
        resetButton.OnClick.AddListener(    ResetAdjustments);
        makeFlatButton.OnClick.AddListener( FlattenTarget);
        scanAgainButton.OnClick.AddListener(imageTrackingBehaviour.enableTracking);
    }

    // Update is called once per frame 
    void Update()
    {
        // Rotation
        if (rotating) {
            target.LookAt(interactor);
            target.Rotate(initialRotationOffset.eulerAngles);
        }

        // Translating
        if (translating) {
            target.position = interactor.position + initialTranslationOffset;
        }

        // Drawing ray from interactor to target
        if (rotating || translating) {
            lineRenderer.positionCount = 2;
            Vector3[] pos = new Vector3[] {interactor.position, target.position};
            lineRenderer.SetPositions(pos);
        } else {
            lineRenderer.positionCount = 0;
            lineRenderer.SetPositions(new Vector3[0]);
        }

        // Interactor orientation
        if (manipulationType == Manip.Rotate) {
            interactor.LookAt(target);
        } else {
            interactor.LookAt(Vector3.forward);
        }
    }

    void SelectManipulationType(Manip type) {

        manipulationType = type;

        rotateInteractor.SetActive(manipulationType == Manip.Rotate);
        translateInteractor.SetActive(manipulationType == Manip.Translate);
        interactor.GetComponent<BoxCollider>().enabled = (manipulationType != Manip.None);

    }

    void ResetAdjustments() {
        target.localPosition = Vector3.zero;
        target.localRotation = Quaternion.identity;
    }

    void FlattenTarget() {
        target.eulerAngles = Vector3.Scale(new Vector3(0,1,0), target.eulerAngles);
    }

    public void BeginManipulation() {
        switch(manipulationType) {

            case Manip.Rotate:
                rotating = true;
                Quaternion initialTargetRotation = target.rotation;
                target.LookAt(interactor);
                initialRotationOffset = Quaternion.Inverse(target.rotation) * (initialTargetRotation);
                target.rotation = initialTargetRotation;
                break;
            
            case Manip.Translate:
                translating = true;
                initialTranslationOffset = target.position - interactor.position;
                break;

            case Manip.None:
                break;

            default:
                break;
        }
    }

    public void EndManipulation() {
        rotating = false;
        translating = false;
    }
}
