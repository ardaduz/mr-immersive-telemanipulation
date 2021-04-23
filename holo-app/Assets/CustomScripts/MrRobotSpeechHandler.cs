using Microsoft.MixedReality.Toolkit.UI;
using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using Microsoft.MixedReality.Toolkit.Input;

using System.Collections;
using UnityEngine.UI;

namespace MrRobot.Input
{
    /// <summary>
    /// This component handles the speech input events raised form the <see cref="IMixedRealityInputSystem"/>.
    /// </summary>
    [DisallowMultipleComponent]
    public class MrRobotSpeechHandler : BaseInputHandler, IMixedRealitySpeechHandler
    {
        /// <summary>
        /// The keywords to be recognized and optional keyboard shortcuts.
        /// </summary>
        public KeywordAndResponse[] Keywords => keywords;

        [SerializeField]
        [Tooltip("The keywords to be recognized and optional keyboard shortcuts.")]
        private KeywordAndResponse[] keywords = new KeywordAndResponse[0];

        [SerializeField]
        [Tooltip("Keywords are persistent across all scenes.  This Speech Input Handler instance will not be destroyed when loading a new scene.")]
        private bool persistentKeywords = false;


        [SerializeField]
        [Tooltip("Assign SpeechConfirmationTooltip.prefab here to display confirmation label. Optional.")]
        private SpeechConfirmationTooltip speechConfirmationTooltipPrefab = null;

        /// <summary>
        /// Tooltip prefab used to display confirmation label. Optional.
        /// </summary>
        public SpeechConfirmationTooltip SpeechConfirmationTooltipPrefab
        {
            get { return speechConfirmationTooltipPrefab; }
            set { speechConfirmationTooltipPrefab = value; }
        }

        private SpeechConfirmationTooltip speechConfirmationTooltipPrefabInstance = null;

        private readonly Dictionary<string, UnityEvent> responses = new Dictionary<string, UnityEvent>();


        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();

            if (persistentKeywords)
            {
                Debug.Assert(gameObject.transform.parent == null, "Persistent keyword GameObject must be at the root level of the scene hierarchy.");
                DontDestroyOnLoad(gameObject);
            }

            // Convert the struct array into a dictionary, with the keywords and the methods as the values.
            // This helps easily link the keyword recognized to the UnityEvent to be invoked.
            int keywordCount = keywords.Length;
            for (int index = 0; index < keywordCount; index++)
            {
                KeywordAndResponse keywordAndResponse = keywords[index];
                string keyword = keywordAndResponse.Keyword.ToLower();

                if (responses.ContainsKey(keyword))
                {
                    Debug.LogError($"Duplicate keyword \'{keyword}\' specified in \'{gameObject.name}\'.");
                }
                else
                {
                    responses.Add(keyword, keywordAndResponse.Response);
                }
            }
        }

        // Update is called once per frame
        protected override void Update()
        {
            base.Update();

        }

        protected override void RegisterHandlers()
        {
            InputSystem?.RegisterHandler<IMixedRealitySpeechHandler>(this);
        }

        protected override void UnregisterHandlers()
        {
            InputSystem?.UnregisterHandler<IMixedRealitySpeechHandler>(this);
        }

        public void AddResponse(string keyword, UnityAction action)
        {
            string lowerKeyword = keyword.ToLower();
            if (!responses.ContainsKey(lowerKeyword))
            {
                responses[lowerKeyword] = new UnityEvent();
            }

            responses[lowerKeyword].AddListener(action);
        }

        public void RemoveResponse(string keyword, UnityAction action)
        {
            string lowerKeyword = keyword.ToLower();
            if (responses.ContainsKey(lowerKeyword))
            {
                responses[lowerKeyword].RemoveListener(action);
            }
        }
        
        void IMixedRealitySpeechHandler.OnSpeechKeywordRecognized(SpeechEventData eventData)
        {
            UnityEvent keywordResponse;
            // Check to make sure the recognized keyword exists in the methods dictionary, then invoke the corresponding method.
            if (enabled && responses.TryGetValue(eventData.Command.Keyword.ToLower(), out keywordResponse))
            {
                keywordResponse.Invoke();
                eventData.Use();

                // Instantiate the Speech Confirmation Tooltip prefab if assigned
                // Ignore "Select" keyword since OS will display the tooltip. 
                if (SpeechConfirmationTooltipPrefab != null
                    && speechConfirmationTooltipPrefabInstance == null
                    && !eventData.Command.Keyword.Equals("select", StringComparison.CurrentCultureIgnoreCase))
                {

                    speechConfirmationTooltipPrefabInstance = Instantiate(speechConfirmationTooltipPrefab);

                    // Update the text label with recognized keyword
                    speechConfirmationTooltipPrefabInstance.SetText(eventData.Command.Keyword);

                    // Trigger animation of the Speech Confirmation Tooltip prefab
                    speechConfirmationTooltipPrefabInstance.TriggerConfirmedAnimation();

                    // Tooltip prefab instance will be destroyed on animation complete 
                    // by DestroyOnAnimationComplete.cs in the SpeechConfirmationTooltip.prefab
                }
            }
        }
    }
}

