from uagents import Agent
from uagents.setup import fund_agent_if_low

agent = Agent(
    name="car",
    seed="Car1",
    port=8002,
    endpoint=["http://127.0.0.1:8002/submit"],
)

fund_agent_if_low(agent.wallet.address())