using UnityEngine;
using CW.Common;

namespace Lean.Touch
{
	/// <summary>This component allows you to translate the current GameObject relative to the camera using the finger drag gesture.</summary>
	[HelpURL(LeanTouch.HelpUrlPrefix + "MY DRAG ROTATE")]
	[AddComponentMenu(LeanTouch.ComponentPathPrefix + "MY DRAG ROTATE")]
	public class DragRotate : MonoBehaviour
	{
		/// <summary>The method used to find fingers to use with this component. See LeanFingerFilter documentation for more information.</summary>
		public LeanFingerFilter Use = new LeanFingerFilter(true);

		/// <summary>The camera the translation will be calculated using.
		/// None/null = MainCamera.</summary>
		public Camera Camera { set { _camera = value; } get { return _camera; } } [SerializeField] private Camera _camera;

		public Transform rotationFocus;

		/// <summary>The movement speed will be multiplied by this.
		/// -1 = Inverted Controls.</summary>
		public float Sensitivity { set { sensitivity = value; } get { return sensitivity; } } [SerializeField] private float sensitivity = 1.0f;

		/// <summary>If you want this component to change smoothly over time, then this allows you to control how quick the changes reach their target value.
		/// -1 = Instantly change.
		/// 1 = Slowly change.
		/// 10 = Quickly change.</summary>
		public float Damping { set { damping = value; } get { return damping; } } [SerializeField] protected float damping = -1.0f;

		/// <summary>This allows you to control how much momentum is retained when the dragging fingers are all released.
		/// NOTE: This requires <b>Dampening</b> to be above 0.</summary>
		public float Inertia { set { inertia = value; } get { return inertia; } } [SerializeField] [Range(0.0f, 1.0f)] private float inertia;

		[SerializeField]
		private Vector3 remainingTranslation;

		/// <summary>If you've set Use to ManuallyAddedFingers, then you can call this method to manually add a finger.</summary>
		public void AddFinger(LeanFinger finger)
		{
			Use.AddFinger(finger);
		}

		/// <summary>If you've set Use to ManuallyAddedFingers, then you can call this method to manually remove a finger.</summary>
		public void RemoveFinger(LeanFinger finger)
		{
			Use.RemoveFinger(finger);
		}

		/// <summary>If you've set Use to ManuallyAddedFingers, then you can call this method to manually remove all fingers.</summary>
		public void RemoveAllFingers()
		{
			Use.RemoveAllFingers();
		}

#if UNITY_EDITOR
		protected virtual void Reset()
		{
			Use.UpdateRequiredSelectable(gameObject);
		}
#endif

		protected virtual void Awake()
		{
			Use.UpdateRequiredSelectable(gameObject);
		}

		Vector3 initialCameraOffset;
		Vector3 cameraOffset;

		void Start() {

			initialCameraOffset = new Vector3(4f, 4f, -6f);
			cameraOffset = initialCameraOffset;

			ResetView();
		}

		public void ResetView() {

			// Reset map transform
			transform.localPosition = Vector3.zero;
			transform.localRotation = Quaternion.identity;
			transform.localScale = new Vector3(1f, 1f, 1f);

			// Reset camera view
			cameraOffset = initialCameraOffset;
			updateCameraPosition();
			_camera.transform.LookAt(rotationFocus);
			
			// Shift camera to centre of point cloud screen area
			cameraOffset = cameraOffset - 2.5f * _camera.transform.right;
			updateCameraPosition();


			
			// _camera.transform.localRotation = Quaternion.identity;
			// _camera.transform.localPosition = new Vector3(0f, 0f, -10f);
			// _camera.transform.RotateAround(rotationFocus.position, _camera.transform.right, 45);
			// _camera.transform.RotateAround(rotationFocus.position, Vector3.up, -45);

			// cameraFollower.setOffset(new Vector3(4f, 4f, -6f));

		}

		protected virtual void Update()
		{
			// Store
			var oldRotation = transform.localEulerAngles;

			// Get the fingers we want to use
			var fingers = Use.UpdateAndGetFingers();

			// Calculate the screenDelta value based on these fingers
			var screenDelta = LeanGesture.GetScreenDelta(fingers);

			// Custom rotation
            Rotate(screenDelta);

			// Camera follows Jackal
			updateCameraPosition();
		}

		private void updateCameraPosition() {
			_camera.transform.position = rotationFocus.transform.position + cameraOffset;
		}

		private void Rotate(Vector2 screenDelta)
		{
			// Make sure the camera exists
			var camera = CwHelper.GetCamera(this._camera, gameObject);

			if (camera != null)
			{
                float factor = 0.2f;
				if (screenDelta.x != 0 || screenDelta.y != 0) {
					updateCameraPosition();
					_camera.transform.RotateAround(rotationFocus.position, Vector3.up, factor*screenDelta.x);
					_camera.transform.RotateAround(rotationFocus.position, _camera.transform.right, -factor*screenDelta.y);
					cameraOffset = _camera.transform.position - rotationFocus.transform.position;
				}
			}
			else
			{
				Debug.LogError("Failed to find camera. Either tag your camera as MainCamera, or set one in this component.", this);
			}
		}
	}
}