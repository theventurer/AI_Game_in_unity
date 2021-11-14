using UnityEngine;
using System.Collections.Generic;
#if UNITY_5_5_OR_NEWER
using UnityEngine.Profiling;
#endif

namespace Pathfinding {
	[AddComponentMenu("Pathfinding/Seeker")]
	public class Seeker : VersionedMonoBehaviour {
		public bool drawGizmos = true;
		public bool detailedGizmos;
		[HideInInspector]
		public StartEndModifier startEndModifier = new StartEndModifier();
		[HideInInspector]
		public int traversableTags = -1;

		[HideInInspector]
		public int[] tagPenalties = new int[32];

		[HideInInspector]
		public GraphMask graphMask = GraphMask.everything;
		[UnityEngine.Serialization.FormerlySerializedAs("graphMask")]
		int graphMaskCompatibility = -1;
		public OnPathDelegate pathCallback;
		public OnPathDelegate preProcessPath;
		public OnPathDelegate postProcessPath;
		[System.NonSerialized]
		List<Vector3> lastCompletedVectorPath;
		[System.NonSerialized]
		List<GraphNode> lastCompletedNodePath;
		[System.NonSerialized]
		protected Path path;
		[System.NonSerialized]
		private Path prevPath;
		private readonly OnPathDelegate onPathDelegate;
		private OnPathDelegate tmpPathCallback;
		protected uint lastPathID;
		readonly List<IPathModifier> modifiers = new List<IPathModifier>();

		public enum ModifierPass {
			PreProcess,
			PostProcess = 2,
		}

		public Seeker () {
			onPathDelegate = OnPathComplete;
		}

		protected override void Awake () {
			base.Awake();
			startEndModifier.Awake(this);
		}
		public Path GetCurrentPath () {
			return path;
		}
		public void CancelCurrentPathRequest (bool pool = true) {
			if (!IsDone()) {
				path.FailWithError("Canceled by script (Seeker.CancelCurrentPathRequest)");
				if (pool) {
					path.Claim(path);
					path.Release(path);
				}
			}
		}

		public void OnDestroy () {
			ReleaseClaimedPath();
			startEndModifier.OnDestroy(this);
		}
		void ReleaseClaimedPath () {
			if (prevPath != null) {
				prevPath.Release(this, true);
				prevPath = null;
			}
		}

		public void RegisterModifier (IPathModifier modifier) {
			modifiers.Add(modifier);
			modifiers.Sort((a, b) => a.Order.CompareTo(b.Order));
		}
		public void DeregisterModifier (IPathModifier modifier) {
			modifiers.Remove(modifier);
		}
		public void PostProcess (Path path) {
			RunModifiers(ModifierPass.PostProcess, path);
		}
		public void RunModifiers (ModifierPass pass, Path path) {
			if (pass == ModifierPass.PreProcess) {
				if (preProcessPath != null) preProcessPath(path);

				for (int i = 0; i < modifiers.Count; i++) modifiers[i].PreProcess(path);
			} else if (pass == ModifierPass.PostProcess) {
				Profiler.BeginSample("Running Path Modifiers");
				if (postProcessPath != null) postProcessPath(path);
				for (int i = 0; i < modifiers.Count; i++) modifiers[i].Apply(path);
				Profiler.EndSample();
			}
		}
		public bool IsDone () {
			return path == null || path.PipelineState >= PathState.Returned;
		}

		void OnPathComplete (Path path) {
			OnPathComplete(path, true, true);
		}
		void OnPathComplete (Path p, bool runModifiers, bool sendCallbacks) {
			if (p != null && p != path && sendCallbacks) {
				return;
			}

			if (this == null || p == null || p != path)
				return;

			if (!path.error && runModifiers) {
				RunModifiers(ModifierPass.PostProcess, path);
			}

			if (sendCallbacks) {
				p.Claim(this);

				lastCompletedNodePath = p.path;
				lastCompletedVectorPath = p.vectorPath;

				// This will send the path to the callback (if any) specified when calling StartPath
				if (tmpPathCallback != null) {
					tmpPathCallback(p);
				}

				// This will send the path to any script which has registered to the callback
				if (pathCallback != null) {
					pathCallback(p);
				}

				if (prevPath != null) {
					prevPath.Release(this, true);
				}

				prevPath = p;
			}
		}

		[System.Obsolete("Use ABPath.Construct(start, end, null) instead")]
		public ABPath GetNewPath (Vector3 start, Vector3 end) {
			// Construct a path with start and end points
			return ABPath.Construct(start, end, null);
		}
		public Path StartPath (Vector3 start, Vector3 end) {
			return StartPath(start, end, null);
		}
		public Path StartPath (Vector3 start, Vector3 end, OnPathDelegate callback) {
			return StartPath(ABPath.Construct(start, end, null), callback);
		}

		public Path StartPath (Vector3 start, Vector3 end, OnPathDelegate callback, GraphMask graphMask) {
			return StartPath(ABPath.Construct(start, end, null), callback, graphMask);
		}
		public Path StartPath (Path p, OnPathDelegate callback = null) {
			if (p.nnConstraint.graphMask == -1) p.nnConstraint.graphMask = graphMask;
			StartPathInternal(p, callback);
			return p;
		}

		public Path StartPath (Path p, OnPathDelegate callback, GraphMask graphMask) {
			p.nnConstraint.graphMask = graphMask;
			StartPathInternal(p, callback);
			return p;
		}

		/// <summary>Internal method to start a path and mark it as the currently active path</summary>
		void StartPathInternal (Path p, OnPathDelegate callback) {
			p.callback += onPathDelegate;

			p.enabledTags = traversableTags;
			p.tagPenalties = tagPenalties;
			if (path != null && path.PipelineState <= PathState.Processing && path.CompleteState != PathCompleteState.Error && lastPathID == path.pathID) {
				path.FailWithError("Canceled path because a new one was requested.\n"+
					"This happens when a new path is requested from the seeker when one was already being calculated.\n" +
					"For example if a unit got a new order, you might request a new path directly instead of waiting for the now" +
					" invalid path to be calculated. Which is probably what you want.\n" +
					"If you are getting this a lot, you might want to consider how you are scheduling path requests.");
			}

			path = p;
			tmpPathCallback = callback;

			lastPathID = path.pathID;
			RunModifiers(ModifierPass.PreProcess, path);

			AstarPath.StartPath(path);
		}

		public void OnDrawGizmos () {
			if (lastCompletedNodePath == null || !drawGizmos) {
				return;
			}

			if (detailedGizmos) {
				Gizmos.color = new Color(0.7F, 0.5F, 0.1F, 0.5F);

				if (lastCompletedNodePath != null) {
					for (int i = 0; i < lastCompletedNodePath.Count-1; i++) {
						Gizmos.DrawLine((Vector3)lastCompletedNodePath[i].position, (Vector3)lastCompletedNodePath[i+1].position);
					}
				}
			}

			Gizmos.color = new Color(0, 1F, 0, 1F);

			if (lastCompletedVectorPath != null) {
				for (int i = 0; i < lastCompletedVectorPath.Count-1; i++) {
					Gizmos.DrawLine(lastCompletedVectorPath[i], lastCompletedVectorPath[i+1]);
				}
			}
		}

		protected override int OnUpgradeSerializedData (int version, bool unityThread) {
			if (graphMaskCompatibility != -1) {
				Debug.Log("Loaded " + graphMaskCompatibility + " " + graphMask.value);
				graphMask = graphMaskCompatibility;
				graphMaskCompatibility = -1;
			}
			return base.OnUpgradeSerializedData(version, unityThread);
		}
	}
}
