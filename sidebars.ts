import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  handbook: [
    'home',
    {
      type: 'category',
      label: '开始使用',
      items: [
        'getting-started/quickstart',
        'getting-started/client',
        'architecture/deployment-and-startup',
        'architecture/multiplatform-deployment',
      ],
    },
    {
      type: 'category',
      label: '接入与发布',
      items: [
        'integration-guide/index',
        'integration-guide/vendor-onboarding',
        'integration-guide/packaging-spec',
        'integration-guide/build-and-codegen',
        'integration-guide/package-catalog',
      ],
    },
    {
      type: 'category',
      label: '软件包开发',
      items: [
        'developer-guide',
        {type: 'doc', id: 'tutorials/existing-python-feature', label: '抓积木接入示例'},
      ],
    },
    {
      type: 'category',
      label: '理解 Robonix',
      items: [
        'architecture/components',
        'architecture/runtime-communication',
        'architecture/namespace-and-contracts',
        'architecture/atlas',
        'background/eaios',
      ],
    },
    {
      type: 'category',
      label: '接口参考',
      items: [
        {type: 'doc', id: 'interface-catalog/index', label: '接口目录'},
        {
          type: 'category',
          label: '原语',
          items: [
            {type: 'doc', id: 'interface-catalog/primitive/index', label: '原语概览'},
            {type: 'doc', id: 'interface-catalog/primitive/chassis', label: '底盘'},
            {type: 'doc', id: 'interface-catalog/primitive/arm', label: '机械臂'},
            {type: 'doc', id: 'interface-catalog/primitive/camera', label: '相机'},
            {type: 'doc', id: 'interface-catalog/primitive/lidar', label: '激光雷达'},
            {type: 'doc', id: 'interface-catalog/primitive/imu', label: '惯性测量单元'},
            {type: 'doc', id: 'interface-catalog/primitive/audio', label: '音频'},
            {type: 'doc', id: 'interface-catalog/primitive/health', label: '设备健康'},
            {type: 'doc', id: 'interface-catalog/primitive/robot-description', label: '机器人描述'},
          ],
        },
        {
          type: 'category',
          label: '服务',
          items: [
            {type: 'doc', id: 'interface-catalog/service/index', label: '服务概览'},
            {type: 'doc', id: 'interface-catalog/service/map', label: '空间地图'},
            {type: 'doc', id: 'interface-catalog/service/navigation', label: '导航'},
            {type: 'doc', id: 'interface-catalog/service/speech', label: '语音'},
            {type: 'doc', id: 'interface-catalog/service/voiceprint', label: '声纹'},
            {type: 'doc', id: 'interface-catalog/service/memory', label: '记忆'},
          ],
        },
        {
          type: 'category',
          label: '技能',
          items: [{type: 'doc', id: 'interface-catalog/skill/index', label: '技能概览'}],
        },
        {
          type: 'category',
          label: '系统',
          items: [
            {type: 'doc', id: 'interface-catalog/system/index', label: '系统概览'},
            {type: 'doc', id: 'interface-catalog/system/soma', label: '本体模型'},
            {type: 'doc', id: 'interface-catalog/system/vitals', label: '健康评估'},
            {type: 'doc', id: 'interface-catalog/system/pilot', label: '规划与决策'},
            {type: 'doc', id: 'interface-catalog/system/executor', label: '任务执行'},
            {type: 'doc', id: 'interface-catalog/system/liaison', label: '用户交互'},
            {type: 'doc', id: 'interface-catalog/system/scene', label: '场景理解'},
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '参考',
      items: [
        'reference/index',
        'reference/contracts',
        'reference/idl',
        'reference/api',
      ],
    },
    {
      type: 'category',
      label: '参与维护',
      items: ['contributing/documentation'],
    },
  ],
};

export default sidebars;
