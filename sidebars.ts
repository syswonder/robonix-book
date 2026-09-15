import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// 类目划分的判据是读者此刻在做什么，不是文件放在哪个目录。architecture/ 下的部署
// 两篇归「部署与运行」，其余归「工作原理」；integration-guide/ 下的打包、构建、
// 发布三篇归「软件包开发」，那里本来就讲怎么做出一个软件包。
//
// 「接口目录」是人写的解释，「自动生成的参考」由 rbnx docs 从源码生成，是权威来源。
// 两个类目的名字要能看出这个区别，否则读者不知道该查哪一个。
const sidebars: SidebarsConfig = {
  handbook: [
    'home',
    {
      type: 'category',
      label: '入门',
      items: [
        'getting-started/quickstart',
        'getting-started/client',
      ],
    },
    {
      type: 'category',
      label: '部署与运行',
      items: [
        'architecture/deployment-and-startup',
        'architecture/multiplatform-deployment',
        {
          type: 'doc',
          id: 'getting-started/x86-ubuntu-la-arch/README',
          label: 'x86 仿真 + LoongArch 部署',
        },
      ],
    },
    {
      type: 'category',
      label: '组件使用指南',
      items: [
        {type: 'doc', id: 'components/rbnx', label: 'rbnx 命令行'},
        {type: 'doc', id: 'components/mapping', label: '建图与定位'},
        {type: 'doc', id: 'components/navigation', label: '导航'},
        {type: 'doc', id: 'components/scene', label: '场景服务'},
        {type: 'doc', id: 'appendix/speech-backends', label: '语音后端配置'},
      ],
    },
    {
      type: 'category',
      label: '接入机器人',
      items: [
        'integration-guide/index',
        'integration-guide/vendor-onboarding',
        {
          type: 'doc',
          id: 'integration-guide/mujoco-simulation-onboarding',
          label: '接入 MuJoCo 仿真本体',
        },
        {type: 'doc', id: 'tutorials/mapping-and-odometry', label: '接入传感器与里程计'},
      ],
    },
    {
      type: 'category',
      label: '软件包开发',
      items: [
        'developer-guide',
        {type: 'doc', id: 'tutorials/existing-python-feature', label: '抓积木接入示例'},
        'integration-guide/packaging-spec',
        'integration-guide/build-and-codegen',
        'integration-guide/package-catalog',
      ],
    },
    {
      type: 'category',
      label: '工作原理',
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
      label: '接口目录',
      items: [
        {type: 'doc', id: 'interface-catalog/index', label: '接口目录总览'},
        {
          type: 'category',
          label: '原语',
          items: [
            {type: 'doc', id: 'interface-catalog/primitive/index', label: '原语概览'},
            {type: 'doc', id: 'interface-catalog/primitive/chassis', label: '底盘'},
            {type: 'doc', id: 'interface-catalog/primitive/arm', label: '机械臂'},
            {type: 'doc', id: 'interface-catalog/primitive/hand', label: '灵巧手'},
            {type: 'doc', id: 'interface-catalog/primitive/quadruped', label: '四足底盘'},
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
      label: '自动生成的参考',
      items: [
        {type: 'doc', id: 'reference/index', label: '生成说明'},
        'reference/contracts',
        'reference/idl',
        'reference/api',
      ],
    },
    {
      type: 'category',
      label: '参与维护',
      items: [
        {type: 'doc', id: 'contributing/robonix', label: '贡献 Robonix 代码'},
        {type: 'doc', id: 'contributing/documentation', label: '维护文档'},
      ],
    },
  ],
};

export default sidebars;
