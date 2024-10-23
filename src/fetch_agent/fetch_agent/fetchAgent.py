from uagents import Agent
from uagents.setup import fund_agent_if_low
import socket

hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)

agent = Agent(
    name="car",
    seed="Car1",
    port=8002,
    endpoint=["http://{}:8002/submit".format(IPAddr)],
)

fund_agent_if_low(agent.wallet.address())