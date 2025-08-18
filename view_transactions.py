from web3 import Web3

# Connect to Ganache
ganache_url = 'http://127.0.0.1:8545'  # Adjust port if using GUI (7545)
web3 = Web3(Web3.HTTPProvider(ganache_url))

def view_transaction_data(tx_hash):
    # Get transaction details
    tx = web3.eth.get_transaction(tx_hash)
    # Decode the 'data' field (assuming it's ASCII-encoded)
    data = tx['data'].decode('ascii') if isinstance(tx['data'], bytes) else web3.to_text(tx['data'])
    print(f"Transaction {tx_hash}:")
    print(f"Data: {data}")

# Example: Replace with your transaction hash from log_data output
tx_hash = '0xbb501b2a644c512e33c7925c7e7605690cc8cfb9d3c49ee2ca083463ca6cfb44'  # e.g., from print(f"Transaction hash: {web3.to_hex(tx_hash)}")
view_transaction_data(tx_hash)