"""
Title: Decision Tree classifier for continuous features and discrete outcomes.

Data Requirements: Numpy array of all feature values (continuous values), Numpy or
                   python array of all target/labels (discrete/categorical values)

Example Data Set: Iris Data Set

Usage:

    #create class
    dt = DecisionTreeCustom()

    #fit data
    dt.fit(X_train, y_train)

    #predict records
    dt.predict(y_test)


Author: Adrian Bisberg
"""
import numpy
import pandas as pd
import numpy as np
import math


class DecisionNode():
    """
    Class to represent a node (decision) in a decision tree. Has a feature, a split point,
    a tree for below the split point, and a tree for above or equal to te split point.
    If the below or above is a decision it will be an integer classification value and not another tree.
    """

    def __init__(self, feature, split, below, above):
        self.feature = feature
        self.split = split
        self.below = below
        self.above = above

    def __str__(self):
        return str(self.feature) + " split at " + str(self.split) + "\nbelow: " + str(self.below) + " above: " + str(self.above) + "\n"

class DecisionTreeCustom():

    """
    Custom decision tree class for continuous features and discrete classification.
    """

    def __init__(self, depth_limit = None):
        self.tree = None
        self.depth_limit = depth_limit

    def fit(self, features, target):
        """
        Fit features and target values to the tree.
        :param features: Numpy array containing rows on all feature attributes to train on.
        :param target: Array of target outcomes for training data.
        """
        d = features.copy()
        t = target.copy()
        df = pd.DataFrame(data=d, columns=d.columns)
        df['target'] = t
        if self.depth_limit is None:
            self.tree = self.__build_tree__(df)
        elif self.depth_limit < 1:
            return "error cannot set depth below 1"
        else:
            self.tree = self.__build_tree__(df, self.depth_limit)

    def predict(self, records):
        """
        Predict an numpy array of records.
        :param records: rows of data containing values for each feature
        :return: the prediction results for all records.
        """
        if(self.tree == None):
            return "Not yet trained cannot predict"
        result = []
        for i in range(records.shape[0]):
            first = records.iloc[i]
            result.append(self.__classify__(first, self.tree))
        return result

    def __str__(self):
        return str(self.tree)

    """
    ============= Auxillary functions ==============
    """

    def __classify__(self, row, tree):
        """
        Classify
        :param row: a record of data, with values for each feature
        :param tree: the root decision node being examined
        :return:
        """
        if(type(tree) == numpy.int64 or type(tree) == int):
            return tree
        root = tree.feature
        split = tree.split
        if row[root] < split:
            return self.__classify__(row, tree.below)
        else:
            return self.__classify__(row, tree.above)

    def __entropy__(self, data):
        """
        Calculate the entropy of the given data frame
        Requiring the target features are specified in a 'target' column
        """
        options = np.unique(data['target'])
        sum = 0
        for i in range(len(options)):
            occurance_count = data.loc[data['target'] == i].shape[0]
            if (occurance_count > 0):
                p_i = data.loc[data['target'] == i].shape[0] / data.shape[0]
                sum += - p_i * math.log2(p_i)
        return sum

    def __gain_at_split__(self, df, first, second, prior_ent):
        """
        Calculate the information gain at a binary split on one column of the data, given that the split is described
        by the first and second half of the data for that split
        """
        # number of elements in df
        total = df.shape[0]

        # entropy of the first side
        ent_one = self.__entropy__(first)

        # entropy of the
        ent_two = self.__entropy__(second)

        combined_ent = ((first.shape[0] / total) * ent_one) + ((second.shape[0] / total) * ent_two)
        gain = prior_ent - combined_ent
        return gain


    def __best_split_gain__(self, data, feature_name):
        """
        Caluclate the best possible gain for this feature and the
        location where it should be split

        :param data: the dataframe with features and target columns
        :param feature_name: the name of the feature to calculate split for
        :return: the best value for gain of this feature and the best location for splitting
        """
        high, low = np.amax(data[feature_name]), np.amin(data[feature_name])
        range = high / low
        split = range / 100
        split_vals = np.arange(low + split, high, split)
        best_gain, best_split = 0, 0
        prior_ent = self.__entropy__(data)
        for split_point in split_vals:
            first = data.loc[data[feature_name] < split_point]
            second = data.loc[data[feature_name] >= split_point]
            gain = self.__gain_at_split__(data, first, second, prior_ent)
            if (gain > best_gain):
                best_gain, best_split = gain, split_point
        return best_gain, best_split

    def __find_best_feature__(self, data):
        """
        Find the best feature given a numpy array of features with training
        data and their target values.
        :param data: training array
        :return: the best variable, the array of features with values below the split,
        the array of features with values equal to or above the split, the value for
        where the continuous feature is split
        """
        cols = data.columns
        best_gain, best_var, best_split = 0, None, 0
        for feature in cols:
            if (feature != 'target'):
                gain, split = self.__best_split_gain__(data, feature)
                if gain > best_gain:
                    best_gain = gain
                    best_var = feature
                    best_split = split
        return best_var, data.loc[data[best_var] < best_split], data.loc[data[best_var] >= best_split], best_split


    def __build_tree__(self, data, limit = float('inf')):
        """
        Recursive build tree function to train on data.
        :param data: numpy array of training data.
        :return: the decision node with the most gain for this data.
        """
        feature, left, right, split = self.__find_best_feature__(data)

        left = left.drop(columns=feature)
        right = right.drop(columns=feature)
        choice = None
        below, above = None, None

        if (len(np.unique(left['target'])) == 1
                or left.columns[0] == 'target'
                or limit == 1):
            below = int(np.bincount(left['target']).argmax())
        else:
            below = self.__build_tree__(left, limit-1)

        if len(np.unique(right['target'])) == 1 \
                or right.columns[0] == 'target'\
                or limit == 1:
            above = int(np.bincount(right['target']).argmax())
        else:
            above = self.__build_tree__(right, limit-1)

        return DecisionNode(feature, split, below, above)
