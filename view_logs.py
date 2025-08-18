from web3 import Web3
import json

# Connect to Ganache
ganache_url = 'http://127.0.0.1:8545'  # Adjust to 7545 for Ganache GUI
web3 = Web3(Web3.HTTPProvider(ganache_url))

# Contract details (from deployment)
abi = json.loads('''[
	{
		"inputs": [
			{
				"internalType": "string",
				"name": "_position",
				"type": "string"
			},
			{
				"internalType": "string",
				"name": "_message",
				"type": "string"
			}
		],
		"name": "addLog",
		"outputs": [],
		"stateMutability": "nonpayable",
		"type": "function"
	},
	{
		"inputs": [
			{
				"internalType": "uint256",
				"name": "index",
				"type": "uint256"
			}
		],
		"name": "getLog",
		"outputs": [
			{
				"internalType": "uint256",
				"name": "",
				"type": "uint256"
			},
			{
				"internalType": "string",
				"name": "",
				"type": "string"
			},
			{
				"internalType": "string",
				"name": "",
				"type": "string"
			}
		],
		"stateMutability": "view",
		"type": "function"
	},
	{
		"inputs": [],
		"name": "getLogCount",
		"outputs": [
			{
				"internalType": "uint256",
				"name": "",
				"type": "uint256"
			}
		],
		"stateMutability": "view",
		"type": "function"
	},
	{
		"inputs": [
			{
				"internalType": "uint256",
				"name": "",
				"type": "uint256"
			}
		],
		"name": "logs",
		"outputs": [
			{
				"internalType": "uint256",
				"name": "timestamp",
				"type": "uint256"
			},
			{
				"internalType": "string",
				"name": "position",
				"type": "string"
			},
			{
				"internalType": "string",
				"name": "message",
				"type": "string"
			}
		],
		"stateMutability": "view",
		"type": "function"
	}
]''')  # Replace with ABI from Remix
contract_address = '0xA3B05D58a27B9FBdFfACd528264242075EB271D8'  # Paste deployed address here for future runs

# Load contract
contract = web3.eth.contract(address=contract_address, abi=abi)

def view_logs():
    # Get total number of logs
    log_count = contract.functions.getLogCount().call()
    print(f"Total logs: {log_count}")

    # Iterate through all logs
    for i in range(log_count):
        timestamp, position, message = contract.functions.getLog(i).call()
        print(f"Log {i}:")
        print(f"  Timestamp: {timestamp} (UTC: {time.ctime(timestamp)})")
        print(f"  Position: {position}")
        print(f"  Message: {message}")

if __name__ == '__main__':
    view_logs()