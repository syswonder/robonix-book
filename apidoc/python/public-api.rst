Robonix Python 公共接口
=======================

本页只列出软件包开发者直接使用的公共入口。签名、默认值和源码链接由
Sphinx 从当前 Robonix 源码生成；概念与完整示例见《Robonix 开发者指南》。

能力提供方
----------

``Primitive``、``Service`` 和 ``Skill`` 共享下面的方法。这里以公开的
``Primitive`` 类展示；三者的区别是向 Atlas 注册的提供方类型，以及技能的
延迟激活行为。

.. autoclass:: robonix_api.Primitive
   :members: on_init, on_activate, on_deactivate, on_shutdown, declare_capability, declare_ros2_topic, declare_ros2_service, declare_grpc, declare_mcp, connect_capability, spawn, wait_for_topic, create_publisher, create_subscription, create_subscription_from_channel, emit, provides_mcp, mcp, use_mcp_app, attach_grpc_servicer, provides_grpc, grpc, bootstrap, run
   :inherited-members:
   :member-order: bysource
   :no-index:

构造函数与类说明见 :class:`robonix_api.Primitive`、
:class:`robonix_api.Service` 和 :class:`robonix_api.Skill`。

能力目录
--------

``robonix_api.ATLAS`` 是公开的能力目录单例。下面列出开发者用于发现能力、
查询能力约定和建立通道的方法。

.. autoclass:: robonix_api.atlas._Atlas
   :members: query, query_primitives, query_services, query_skills, find_capability, find_unique_capability, query_contract, list_contracts, connect_capability, disconnect_capability, inspect
   :member-order: bysource

数据类型
--------

返回值和枚举见 :mod:`robonix_api.atlas_types`，包括
:class:`~robonix_api.atlas_types.Capability`、
:class:`~robonix_api.atlas_types.CapabilityProvider`、
:class:`~robonix_api.atlas_types.Channel`、
:class:`~robonix_api.atlas_types.Transport` 和
:class:`~robonix_api.atlas_types.LifecycleState`。
