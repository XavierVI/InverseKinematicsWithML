import torch
from torch import nn, optim
from torch.utils.data import TensorDataset, DataLoader

import pandas as pd

import argparse



class MLPModel(nn.Module):
    def __init__(self):
        super(MLPModel, self).__init__()
        self.hidden_layer = nn.Linear(3, 100)
        self.output_layer = nn.Linear(100, 3)
        self.activation_function = nn.Tanh()

    def forward(self, x):
        x = self.activation_function(self.hidden_layer(x))
        x = self.output_layer(x)
        return x


def train_one_epoch(dataloader, model, optimizer, mse):
    # the running loss is the mean of the losses accumulated
    # from a batch
    running_loss = 0
    last_loss = 0
    for batch_idx, (inputs, targets) in enumerate(dataloader):
        # zero out the gradients
        optimizer.zero_grad()
        
        outputs = model(inputs)
        
        # computes the loss and its gradients
        loss = mse(outputs, targets)
        loss.backward()
        
        # adjust weights
        optimizer.step()
        
        running_loss += loss.item()

        if batch_idx % 32 == 31:
            # compute the mean of the running_loss
            last_loss = running_loss / 32
            print(f'  batch {batch_idx} loss: {last_loss}')
            running_loss = 0.
    
    return last_loss



def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--csv_file', default=None)
    args = p.parse_args()

    """
    Loading the data
    """
    dataframe = pd.read_csv(args.csv_file)

    """
    Training loop
    """
    model = MLPModel()
    model.double()

    # transforming data into tensors
    X_train_tensor = torch.from_numpy(X_train)
    y_train_tensor = torch.from_numpy(y_train)
    X_test_tensor = torch.from_numpy(X_test)
    y_test_tensor = torch.from_numpy(y_test)

    dataset = TensorDataset(X_train_tensor, y_train_tensor)
    test_dataset = TensorDataset(X_test_tensor, y_test_tensor)

    dataloader = DataLoader(dataset,       batch_size=32, shuffle=False)
    test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)


    mse = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    epochs = 10

    for epoch in range(epochs):
        print(f'Epoch: {epoch+1}')
        model.train(True)
        avg_loss = train_one_epoch(dataloader, model, optimizer, mse)

        running_vloss = 0

        model.eval()

        with torch.no_grad():
            for i, vdata in enumerate(test_dataset):
                vinputs, vlabels = vdata
                voutputs = model(vinputs)
                vloss = mse(voutputs, vlabels)
                running_vloss += vloss

        avg_vloss = running_vloss / (i + 1)
        print(f'LOSS train {avg_loss} valid {avg_vloss}')

        print(f"Epoch [{epoch + 1}/{epochs}], Loss: {running_vloss / len(dataloader):.4f}")

if __name__ == '__main__':
    main()
