# -*- coding: utf-8 -*-
"""港口迷雾多车协同模块（负责人：D）。

接口：metacar_multiagent.ScenarioTaskAPI（端口 9100）。
任务动作：submit_key / report_observation / scan / request_entry / release / deliver
每次 interact 之后都必须检查返回的 granted。
"""
