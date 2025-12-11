const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module I: Mathematical Foundations',
      collapsed: false,
      items: [
        'module-1/linear-algebra',
        'module-1/kinematics-dynamics',
        'module-1/control-theory'
      ]
    },
    {
      type: 'category',
      label: 'Module II: The Nervous System',
      collapsed: false,
      items: [
        'module-2/ros2-architecture',
        'module-2/dds-middleware',
        'module-2/realtime-systems',
        'module-2/cpp-lifecycle'
      ]
    },
    {
      type: 'category',
      label: 'Module III: The Digital Twin',
      collapsed: false,
      items: [
        'module-3/physics-engines',
        'module-3/urdf-sdf-xacro',
        'module-3/gazebo-plugins',
        'module-3/unity-integration'
    ]},
    {
      type: 'category',
      label: 'Module IV: Robot Perception',
      collapsed: false,
      items: [
        'module-4/visual-slam',
        'module-4/sensor-fusion',
        'module-4/isaac-ros-gems',
        'module-4/point-clouds'
    ]},
    {
      type: 'category',
      label: 'Module V: Cognitive Robotics',
      collapsed: false,
      items: [
        'module-5/generative-ai',
        'module-5/transformers',
        'module-5/voice-pipeline',
        'module-5/safety-alignment',
        'module-5/capstone-project'
      ]
    }
  ],
};
export default sidebars;