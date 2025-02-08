from torch.utils.data import TensorDataset, DataLoader
import torch
from torch import nn, optim
from sklearn.model_selection import train_test_split
import pandas as pd


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


def train_one_epoch():
    # the running loss is the mean of the losses accumulated
    # from a batch
    running_loss = 0
    last_loss = 0
    for i, (inputs, targets) in enumerate(training_dataloader):
        # zeros out the gradients
        optimizer.zero_grad()
        
        # make predictions for this batch
        outputs = model(inputs)
        
        # computes the loss and its gradients
        loss = mse(outputs, targets)
        loss.backward()
        
        # adjust learning weights
        optimizer.step()
        
        running_loss += loss.item()
        # every 1000 batches, compute the mean of the loss
        if i % 1000 == 999:
            # compute the mean of the running_loss
            last_loss = running_loss / 1000
            print(f'  batch {i + 1} loss: {last_loss}')
            running_loss = 0.
    
    return last_loss


# initializing the model and loading the dataset
model = MLPModel()

path = './data.csv'
data = pd.read_csv(path)

X = data[['x','y','pitch']].to_numpy()
y = data[['shoulder','elbow','wrist']].to_numpy()

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
X_val, X_test, y_val, y_test = train_test_split(X_test, y_test, test_size=0.5, random_state=42)

# transforming data into tensors
X_train_tensor = torch.tensor(X_train, dtype=torch.float64)
y_train_tensor = torch.tensor(y_train, dtype=torch.float64)
X_val_tensor   = torch.tensor(X_val,   dtype=torch.float64)
y_val_tensor   = torch.tensor(y_val,   dtype=torch.float64)

training_dataset   = TensorDataset(X_train_tensor, y_train_tensor)
validation_dataset = TensorDataset(X_val_tensor, y_val_tensor)

training_dataloader   = DataLoader(training_dataset,   batch_size=4, shuffle=False)
validation_dataloader = DataLoader(validation_dataset, batch_size=4, shuffle=False)


mse = nn.MSELoss()
optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
# optimizer = optim.Adam(model.parameters(), lr=0.001)

epochs = 150

avg_training_losses = []
avg_validation_losses = []

for epoch in range(epochs):
    print(f'Epoch: {epoch+1}')
    model.train(True)
    avg_loss = train_one_epoch()
    running_vloss = 0

    model.eval()

    with torch.no_grad():
        for i, vdata in enumerate(validation_dataloader):
            vinputs, vlabels = vdata
            voutputs = model(vinputs)
            vloss = mse(voutputs, vlabels)
            running_vloss += vloss
    
    avg_vloss = running_vloss / (i + 1)

    print(f'LOSS train {avg_loss} valid {avg_vloss}')
    print(f"Epoch [{epoch + 1}/{epochs}], Loss: {running_vloss / len(training_dataloader):.4f}")

    avg_training_losses.append(avg_loss)
    avg_validation_losses.append(avg_vloss)
