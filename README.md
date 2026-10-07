# goal-action-planner-go

一个目标导向的行动规划与执行运行时（GOAP 风格）。智能体依据当前世界状态与自身目标检索行动序列；世界在执行过程中持续变化、行动可能失败；规划与执行结果在固定种子和相同输入序列下**逐帧可复现**，便于复盘与调参。

## 快速开始

```go
actions := goap.NewActionSet(
    &goap.Action{Name: "chop",  Cost: 1, Effects: []goap.Effect{{Key: "wood", Value: true}}},
    &goap.Action{Name: "build", Cost: 1,
        Preconditions: []goap.Condition{{Key: "wood", Value: true}},
        Effects:       []goap.Effect{{Key: "house", Value: true}}},
)
planner  := goap.NewPlanner(actions, goap.PlannerConfig{})
world    := goap.NewWorld(goap.State{})
executor := goap.NewExecutor(planner, nil, goap.ExecutorConfig{})

res := executor.Run(context.Background(), "agent-1", world,
    []goap.Condition{{Key: "house", Value: true}}, nil)
// res.Outcome == goap.OutcomeSuccess, res.Executed == ["chop", "build"]
```

## 行动模型

| 字段 | 语义 |
|---|---|
| `Name` | 唯一标识；所有迭代均按名字字典序进行，保证确定性 |
| `Cost` | 非负代价；规划器最小化总代价 |
| `Preconditions` | 等值条件列表，全部满足行动才可用 |
| `Effects` | 效果列表，行动完成时**原子**写入世界 |
| `Mutex` | 互斥组：同一组内的行动不得出现在同一份计划中 |
| `Resources` | 稀缺资源：执行期间需持有，争用由运行时确定性仲裁 |

世界状态是 `map[string]any` 的事实集合，由带版本号的 `World` 持有。
每次成功修改都会递增版本号；`ApplyIfVersion` 提供 CAS 语义，让
规划/执行能发现自己基于的快照已过期。

## 规划语义

- **搜索**：一致代价搜索（启发式为 0 的 A*），从当前状态前向扩展，
  返回总代价最小的行动序列。
- **等价路径取舍**：总代价相同的多条路径中，行动名序列字典序最小者
  胜出；再相同则按入队顺序。规则明确且与行动注册顺序、map 遍历
  顺序无关。
- **终止性**：每个可达状态至多扩展一次，且受 `MaxIterations` /
  `MaxDepth` 预算约束，绝不死循环。预算耗尽返回 `ErrSearchLimit`，
  搜索空间耗尽返回 `ErrNoPlan`，二者可区分。

### 多目标

`PlanGoals` 按 **优先级降序、名字字典序** 的确定顺序评估目标，并把
可行目标**顺序合并**：每个目标的计划在前序已承诺目标产生的状态之上
继续规划，拼接后即一份一致的组合计划。每个目标的结论可区分：

| 结论 | 含义 |
|---|---|
| `satisfied` | 期望事实已成立，无需计划 |
| `planned` | 找到可行行动序列 |
| `unreachable` | 现有行动在预算内无法达成 |
| `conflicting` | 与已承诺的更高/同等优先级目标的事实相矛盾 |
| `deferred` | 依赖（`DependsOn`）的目标本轮未满足/未规划 |

## 规划与执行分离

- 规划基于**快照**进行；世界版本号让过期状态无处遁形。
- 执行器在**每一步之前**用新鲜快照重新校验前置条件；失配时记录
  `stale_state` 事件并**从中断点重规划**，绝不沿用过期计划。
- 效果已通过 CAS 落地的行动**不会重复执行**；效果未达预期或前提被
  破坏时，重规划从当前真实状态出发，**不会跳过仍必需的前置条件**。
- 重规划次数受 `ExecutorConfig.MaxReplans` 限制，超出返回
  `OutcomeFailed`。

## 多智能体与资源争用

`Runtime` 以 tick 推进：每个 tick 内智能体按 **ID 字典序**依次行动，
因此结果与 goroutine 调度无关。稀缺资源采用 **tick 租约**：

- 获取成功即持有至当前 tick 结束（默认租约 1，可配置）；
- 争用时先到先得（按确定的回合顺序），失败方的
  `resource_wait` 事件会记录持有者与租约到期 tick，**胜负可解释**；
- 租约必然到期，中断的智能体**不会留下悬挂占用**；
- 慢智能体的回合受 `TurnTimeout` 限制，超时即在原子步骤边界被中止，
  不会阻塞其他智能体。

## 取消、超时与恢复

- `context.Context` 贯穿规划与执行；取消只发生在**原子步骤之间**，
  不存在"执行了一半"的行动。
- 已落地的效果保留在世界中；用新 context 恢复执行时，执行器从当前
  状态重规划，**最终状态与未中断运行一致**（见
  `TestRuntimeInterruptResumeAlignment`）。

## 确定性保证

固定随机种子 + 相同输入序列 ⇒ 规划路径、行动选择、执行结果与事件日志
逐帧一致：

- 所有集合遍历按字典序进行；
- 等价路径取舍规则明确（见上）；
- 运行时按 tick + 智能体 ID 顺序推进；
- `Handle.Rand()` 提供由 `(seed, agentID, tick)` 派生的确定性随机流。

## 配置项

| 配置 | 默认 | 说明 |
|---|---|---|
| `PlannerConfig.MaxIterations` | 10000 | 单次搜索的最大状态扩展数 |
| `PlannerConfig.MaxDepth` | 64 | 计划最大长度 |
| `ExecutorConfig.MaxReplans` | 8 | 一次执行允许的最大重规划次数 |
| `RuntimeConfig.TurnTimeout` | 0（不限） | 单个智能体回合的超时 |
| `RuntimeConfig.ResourceLeaseTicks` | 1 | 资源租约持续的 tick 数 |
| `NewRuntime` 的 `seed` | — | 运行时级确定性随机种子 |

## 测试

```sh
go test ./...          # 全部测试
go test ./... -race    # 竞态检测
```

覆盖：等价路径取舍、代价优先于字典序、互斥约束、目标冲突/依赖/合并、
环境漂移与行动失败后的重规划、版本冲突、取消与恢复对齐、资源争用的
确定性与可解释性、慢智能体隔离、多次运行逐帧一致。
