#A class for implementing a bot that used multinomial naive bayes to answer simple chat questions
#using training data and answers given to the bot
#
#Use formulate_response(input_text) to recieve a response to a query
#
#Author Adrian Bisberg
import json
import re
from collections import defaultdict
import numpy as np

class NaiveBayesBot:

    def __init__(self):
        #Set up data for training
        queries, stop_words = {}, set()
        responses = {}

        with open('chatbot_data.json') as file:
            data = json.load(file)

        for line in open('stop_words.txt'):
            words = line.split()
            if not re.match('^#',words[0]):
                stop_words.add(words[0])

        for intention in data['intents']:
            for pattern in intention['patterns']:
                pattern = re.sub(r'[^\w\s]', '', pattern)
                contents = pattern.lower().strip().split(" ")
                label = intention['tag']

                usable_words = list(set(contents) - stop_words)  #remove common stop words from the list

                if label not in queries.keys():
                    queries[label] = [usable_words]
                else:
                    queries[label].append(usable_words)
            responses[intention['tag']] = intention['responses']

        output = self.__prepare_data__(queries)

        self.trained_words = output[0]
        self.total_counts = output[1]
        self.class_counts = output[2]
        self.answers = responses

        #Print testing results
        self.test_chat('test_chats.json')

    def test_chat(self, test_file):
        with open(test_file) as file:
            data = json.load(file)

        test_inputs = {}
        responses = {}

        correct = 0
        total = 0

        for intention in data['intents']:
            for pattern in intention['inputs']:
                if intention['tag'] not in test_inputs.keys():
                    test_inputs[intention['tag']] = [pattern]
                if intention['tag'] not in responses.keys():
                    responses[intention['tag']] = [pattern]
                else:
                    test_inputs[intention['tag']].append(pattern)
                    for response in intention['responses']:
                        responses[intention['tag']].append(response)

        for key in test_inputs.keys():
            for sentence in test_inputs[key]:
                total += 1
                answer = self.formulate_response(sentence)
                if responses[key].__contains__(answer):
                    correct += 1

        print("Naive Bayes correctly predicted: ", correct, " out of ", total, " test seneteces")


    #get relevent counts for words across classes, total appearence of words, and total words in classes
    def __prepare_data__(self, queries):
        total_words, class_counts,  total_counts = 0, defaultdict(), defaultdict()
        for key in queries.keys():
            total_class, counts = 0, {}
            for sentence in queries[key]:
                for word in sentence:
                    total_class += 1
                    total_words += 1
                    if word not in counts.keys():
                        counts[word] = 1
                    if word not in total_counts.keys():
                        total_counts[word] = 1
                    else:
                        counts[word] += 1
                        total_counts[word] += 1
            class_counts[key] = total_class, counts
        return total_words,total_counts,class_counts

    #calculates the bayes distribution of a given word
    def __bayes_distribution__(self, sentence):
        count_dict, p_of_class = self.class_counts, defaultdict()
        for class_name in count_dict.keys():
            for word in sentence:
                count = self.class_counts[class_name][1].get(word)
                if count != None:
                    prob_word_given_class = count / self.total_counts[word]
                    if class_name in p_of_class.keys():
                        p_of_class[class_name] *= prob_word_given_class
                    else:
                        p_of_class[class_name] = prob_word_given_class
            if class_name in p_of_class.keys():
                p_of_class[class_name] *= (self.class_counts[class_name][0] / self.trained_words)
        return p_of_class

    def formulate_response(self, input_text):

        input_text = re.sub(r'[^\w\s]', '', input_text)
        split_text = list(input_text.strip().split(" "))
        cumulative = defaultdict()

        #get the bayes distribution of each word
        answer_probs = self.__bayes_distribution__(split_text)

        #accumulate the probabilities for each word
        for key in answer_probs:
            if key not in cumulative.keys():
                cumulative[key] = answer_probs[key]
            else:
                if cumulative[key] == 0.0:
                    cumulative[key] = answer_probs[key]
                else:
                    cumulative[key] = cumulative[key] * answer_probs[key]
        best_val = 0
        best = None

        for key in cumulative: #searching for the maximum key
            if cumulative[key] > best_val:
                best = key
                best_val = cumulative[key]
        if best == None:
            return "I'm sorry I don't understand, please ask again"
        return np.random.choice(self.answers.__getitem__(best)) #answer with the best match


